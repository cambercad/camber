using GeoCore;

namespace Curves
{
    internal sealed class TrueTypeFontFile
    {
        private const int MaxCompositeDepth = 16;
        private const int OnCurvePoint = 0x01;
        private const int XShortVector = 0x02;
        private const int YShortVector = 0x04;
        private const int RepeatFlag = 0x08;
        private const int XIsSameOrPositive = 0x10;
        private const int YIsSameOrPositive = 0x20;

        private const int Arg1And2AreWords = 0x0001;
        private const int ArgsAreXyValues = 0x0002;
        private const int WeHaveAScale = 0x0008;
        private const int MoreComponents = 0x0020;
        private const int WeHaveAnXAndYScale = 0x0040;
        private const int WeHaveATwoByTwo = 0x0080;
        private const int WeHaveInstructions = 0x0100;
        private const int ScaledComponentOffset = 0x0800;

        private readonly TrueTypeBinary _reader;
        private readonly TrueTypeTableDirectory _directory;
        private TrueTypeCmap _cmap;
        private ushort[] _advanceWidths;
        private int[] _loca;
        private TrueTypeGlyphOutline[] _glyphs;
        private readonly object _glyphLock = new object();

        public TrueTypeNameInfo Names { get; private set; }
        public bool Bold { get; private set; }
        public bool Italic { get; private set; }
        public int UnitsPerEm { get; private set; }
        public double Ascent { get; private set; }
        public double Descent { get; private set; }
        public double LineHeight { get; private set; }
        public int GlyphCount { get; private set; }

        private TrueTypeFontFile(TrueTypeBinary reader, TrueTypeTableDirectory directory)
        {
            _reader = reader;
            _directory = directory;
        }

        public static bool TryOpen(byte[] data, int faceIndex, out TrueTypeFontFile font, out TrueTypeFontRef catalogRef)
        {
            font = null;
            catalogRef = null;
            var reader = new TrueTypeBinary(data);
            if (!TrueTypeTableDirectory.TryReadCollectionOffsets(reader, out var offsets))
                return false;
            if (faceIndex < 0 || faceIndex >= offsets.Count)
                return false;
            if (!TrueTypeTableDirectory.TryRead(reader, offsets[faceIndex], out var directory))
                return false;
            if (!directory.HasGlyf)
                return false;

            var parsed = new TrueTypeFontFile(reader, directory);
            if (!parsed.TryInitialize())
                return false;

            font = parsed;
            catalogRef = parsed.ToRef(string.Empty, faceIndex, offsets[faceIndex]);
            return true;
        }

        public int GetGlyphId(char c)
        {
            return _cmap != null ? _cmap.GetGlyphId(c) : 0;
        }

        public bool HasCharacter(char c)
        {
            return _cmap != null && _cmap.GetGlyphId(c) != 0;
        }

        public double GetAdvanceWidth(char c)
        {
            if (char.IsWhiteSpace(c) && c != ' ' && c != '\t')
            {
                if (c == '\r' || c == '\n')
                    return 0;
            }

            int glyphId = HasCharacter(c) ? GetGlyphId(c) : 0;
            if (c == ' ' || c == '\t')
                glyphId = HasCharacter(' ') ? GetGlyphId(' ') : glyphId;

            return GetAdvanceWidthForGlyph(glyphId);
        }

        public TrueTypeGlyphOutline GetOutline(char c)
        {
            if (!HasCharacter(c))
                return EmptyOutline(0);

            return GetOutlineForGlyph(GetGlyphId(c));
        }

        public List<List<Curve2D>> GetContours(char c, Vec2D origin, double emSize)
        {
            var result = new List<List<Curve2D>>();
            if (char.IsWhiteSpace(c))
                return result;

            var outline = GetOutline(c);
            double scale = emSize / Math.Max(1, UnitsPerEm);
            foreach (var contour in outline.Contours)
            {
                var curves = ContourToCurves(contour, origin, scale);
                if (curves.Count > 0)
                    result.Add(curves);
            }

            return result;
        }

        private TrueTypeFontRef ToRef(string path, int faceIndex, int sfntOffset)
        {
            return new TrueTypeFontRef
            {
                Path = path,
                FaceIndex = faceIndex,
                SfntOffset = sfntOffset,
                Names = Names,
                Bold = Bold,
                Italic = Italic,
                HasGlyf = true,
            };
        }

        private bool TryInitialize()
        {
            if (!TryReadIdentity())
                return false;
            if (!TryReadMaxpAndLoca())
                return false;
            if (!TryReadMetrics())
                return false;
            if (!TryReadCmap())
                return false;
            _glyphs = new TrueTypeGlyphOutline[GlyphCount];
            return true;
        }

        private bool TryReadIdentity()
        {
            Names = ReadNameTable();
            if (Names == null || string.IsNullOrWhiteSpace(Names.BestFamily))
                Names = new TrueTypeNameInfo { Family = "Unknown" };

            UnitsPerEm = 1000;
            int macStyle = 0;
            TrueTypeTableRecord head;
            if (_directory.TryGet(TrueTypeBinary.Tag("head"), out head) && head.Length >= 54)
            {
                uint magic = _reader.ReadUInt32At(head.Offset + 12);
                if (magic != 0x5F0F3CF5)
                    return false;
                int units = _reader.ReadUInt16At(head.Offset + 18);
                if (units > 0)
                    UnitsPerEm = units;
                macStyle = _reader.ReadUInt16At(head.Offset + 44);
            }

            bool os2Bold = false;
            bool os2Italic = false;
            TrueTypeTableRecord os2;
            if (_directory.TryGet(TrueTypeBinary.Tag("OS/2"), out os2) && os2.Length >= 6)
            {
                int weight = _reader.ReadUInt16At(os2.Offset + 4);
                os2Bold = weight >= 600;
                if (os2.Length >= 64)
                {
                    int selection = _reader.ReadUInt16At(os2.Offset + 62);
                    os2Bold = os2Bold || (selection & 0x20) != 0;
                    os2Italic = (selection & 0x01) != 0 || (selection & 0x200) != 0;
                }
            }

            string sub = Names.Subfamily ?? string.Empty;
            Bold = os2Bold
                || (macStyle & 0x01) != 0
                || ContainsWord(sub, "Bold")
                || ContainsWord(sub, "Black")
                || ContainsWord(sub, "Heavy");
            Italic = os2Italic
                || (macStyle & 0x02) != 0
                || ContainsWord(sub, "Italic")
                || ContainsWord(sub, "Oblique");
            return true;
        }

        private bool TryReadMaxpAndLoca()
        {
            TrueTypeTableRecord maxp;
            if (!_directory.TryGet(TrueTypeBinary.Tag("maxp"), out maxp) || maxp.Length < 6)
                return false;

            GlyphCount = _reader.ReadUInt16At(maxp.Offset + 4);
            if (GlyphCount <= 0 || GlyphCount > 65535)
                return false;

            TrueTypeTableRecord head;
            TrueTypeTableRecord loca;
            if (!_directory.TryGet(TrueTypeBinary.Tag("head"), out head) || head.Length < 54)
                return false;
            if (!_directory.TryGet(TrueTypeBinary.Tag("loca"), out loca))
                return false;

            int indexToLocFormat = _reader.ReadInt16At(head.Offset + 50);
            _loca = new int[GlyphCount + 1];
            if (indexToLocFormat == 0)
            {
                if (loca.Length < 2 * (_loca.Length))
                    return false;
                for (int i = 0; i < _loca.Length; i++)
                    _loca[i] = _reader.ReadUInt16At(loca.Offset + 2 * i) * 2;
            }
            else
            {
                if (loca.Length < 4 * _loca.Length)
                    return false;
                for (int i = 0; i < _loca.Length; i++)
                    _loca[i] = (int)_reader.ReadUInt32At(loca.Offset + 4 * i);
            }

            return true;
        }

        private bool TryReadMetrics()
        {
            TrueTypeTableRecord hhea;
            if (!_directory.TryGet(TrueTypeBinary.Tag("hhea"), out hhea) || hhea.Length < 36)
                return false;

            int ascent = _reader.ReadInt16At(hhea.Offset + 4);
            int descent = _reader.ReadInt16At(hhea.Offset + 6);
            int lineGap = _reader.ReadInt16At(hhea.Offset + 8);
            int numberOfHMetrics = _reader.ReadUInt16At(hhea.Offset + 34);
            if (numberOfHMetrics <= 0 || numberOfHMetrics > GlyphCount)
                numberOfHMetrics = GlyphCount;

            Ascent = Math.Abs(ascent);
            Descent = Math.Abs(descent);
            LineHeight = Ascent + Descent + Math.Max(0, lineGap);
            if (LineHeight <= 0)
                LineHeight = UnitsPerEm;

            TrueTypeTableRecord os2;
            if (_directory.TryGet(TrueTypeBinary.Tag("OS/2"), out os2) && os2.Length >= 72)
            {
                int typoAsc = _reader.ReadInt16At(os2.Offset + 68);
                int typoDesc = _reader.ReadInt16At(os2.Offset + 70);
                int typoGap = os2.Length >= 74 ? _reader.ReadInt16At(os2.Offset + 72) : 0;
                int fsSelection = os2.Length > 63 ? _reader.ReadUInt16At(os2.Offset + 62) : 0;
                bool useTypo = (fsSelection & 0x80) != 0 && typoAsc != 0;
                if (useTypo)
                {
                    Ascent = Math.Abs(typoAsc);
                    Descent = Math.Abs(typoDesc);
                    LineHeight = Ascent + Descent + Math.Max(0, typoGap);
                }
                else if (os2.Length >= 74)
                {
                    int winAsc = _reader.ReadUInt16At(os2.Offset + 74);
                    int winDesc = os2.Length >= 76 ? _reader.ReadUInt16At(os2.Offset + 76) : 0;
                    if (winAsc > 0)
                    {
                        Ascent = winAsc;
                        Descent = winDesc;
                        LineHeight = Ascent + Descent + Math.Max(0, lineGap);
                    }
                }
            }

            TrueTypeTableRecord hmtx;
            if (!_directory.TryGet(TrueTypeBinary.Tag("hmtx"), out hmtx))
                return false;

            _advanceWidths = new ushort[GlyphCount];
            int lastAdvance = 0;
            int metricCount = Math.Min(numberOfHMetrics, GlyphCount);
            int needed = metricCount * 4;
            if (hmtx.Length < needed)
                return false;

            for (int i = 0; i < metricCount; i++)
            {
                lastAdvance = _reader.ReadUInt16At(hmtx.Offset + i * 4);
                _advanceWidths[i] = (ushort)lastAdvance;
            }

            for (int i = metricCount; i < GlyphCount; i++)
                _advanceWidths[i] = (ushort)lastAdvance;

            return true;
        }

        private bool TryReadCmap()
        {
            TrueTypeTableRecord cmap;
            if (!_directory.TryGet(TrueTypeBinary.Tag("cmap"), out cmap) || cmap.Length < 4)
                return false;

            int tableStart = cmap.Offset;
            int numTables = _reader.ReadUInt16At(tableStart + 2);
            int bestScore = -1;
            int bestOffset = -1;
            int bestFormat = -1;

            for (int i = 0; i < numTables; i++)
            {
                int rec = tableStart + 4 + i * 8;
                if (rec + 8 > tableStart + cmap.Length)
                    break;

                int platform = _reader.ReadUInt16At(rec);
                int encoding = _reader.ReadUInt16At(rec + 2);
                uint subOffset = _reader.ReadUInt32At(rec + 4);
                if (subOffset >= (uint)cmap.Length)
                    continue;

                int absOffset = tableStart + (int)subOffset;
                if (absOffset + 2 > _reader.Length)
                    continue;

                int format = _reader.ReadUInt16At(absOffset);
                int score = ScoreCmap(platform, encoding, format);
                if (score > bestScore)
                {
                    bestScore = score;
                    bestOffset = absOffset;
                    bestFormat = format;
                }
            }

            if (bestOffset < 0)
                return false;

            _cmap = TrueTypeCmap.TryRead(_reader, bestOffset, bestFormat);
            return _cmap != null;
        }

        private static int ScoreCmap(int platform, int encoding, int format)
        {
            if (format != 4 && format != 12 && format != 6 && format != 0)
                return -1;

            int score = format == 12 ? 40 : format == 4 ? 30 : 10;
            if (platform == 3 && encoding == 10)
                score += 50;
            else if (platform == 3 && encoding == 1)
                score += 40;
            else if (platform == 0 && encoding == 4)
                score += 35;
            else if (platform == 0)
                score += 20;
            else if (platform == 3)
                score += 10;
            return score;
        }

        private TrueTypeNameInfo ReadNameTable()
        {
            TrueTypeTableRecord name;
            if (!_directory.TryGet(TrueTypeBinary.Tag("name"), out name) || name.Length < 6)
                return new TrueTypeNameInfo();

            int format = _reader.ReadUInt16At(name.Offset);
            int count = _reader.ReadUInt16At(name.Offset + 2);
            int stringOffset = _reader.ReadUInt16At(name.Offset + 4);
            if (format > 1 || count < 0 || count > 4096)
                return new TrueTypeNameInfo();

            int storage = name.Offset + stringOffset;
            var best = new Dictionary<int, NamePick>();
            for (int i = 0; i < count; i++)
            {
                int rec = name.Offset + 6 + i * 12;
                if (rec + 12 > name.Offset + name.Length)
                    break;

                int platform = _reader.ReadUInt16At(rec);
                int encoding = _reader.ReadUInt16At(rec + 2);
                int language = _reader.ReadUInt16At(rec + 4);
                int nameId = _reader.ReadUInt16At(rec + 6);
                int length = _reader.ReadUInt16At(rec + 8);
                int offset = _reader.ReadUInt16At(rec + 10);
                if (length <= 0 || storage + offset + length > name.Offset + name.Length)
                    continue;
                if (nameId != 1 && nameId != 2 && nameId != 4 && nameId != 16 && nameId != 17)
                    continue;

                string text = DecodeName(platform, encoding, storage + offset, length);
                if (string.IsNullOrWhiteSpace(text))
                    continue;

                int score = ScoreName(platform, encoding, language);
                NamePick existing;
                if (!best.TryGetValue(nameId, out existing) || score > existing.Score)
                    best[nameId] = new NamePick { Score = score, Text = text.Trim() };
            }

            var info = new TrueTypeNameInfo();
            info.Family = Pick(best, 1);
            info.Subfamily = Pick(best, 2);
            info.FullName = Pick(best, 4);
            info.TypographicFamily = Pick(best, 16);
            if (best.ContainsKey(17))
                info.Subfamily = Pick(best, 17);
            return info;
        }

        private string DecodeName(int platform, int encoding, int offset, int length)
        {
            bool unicode = platform == 0 || platform == 3 || (platform == 2 && encoding == 1);
            if (unicode)
            {
                int charCount = length / 2;
                var chars = new char[charCount];
                for (int i = 0; i < charCount; i++)
                    chars[i] = (char)_reader.ReadUInt16At(offset + 2 * i);
                return new string(chars);
            }

            var ascii = new char[length];
            for (int i = 0; i < length; i++)
                ascii[i] = (char)_reader.Data[offset + i];
            return new string(ascii);
        }

        private static int ScoreName(int platform, int encoding, int language)
        {
            int score = 0;
            if (platform == 3 && (encoding == 1 || encoding == 10))
                score += 50;
            else if (platform == 0)
                score += 40;
            else if (platform == 3)
                score += 20;
            else if (platform == 1)
                score += 10;

            if (language == 0x0409)
                score += 20;
            else if ((language & 0xFF) == 0x09)
                score += 10;
            else if (language == 0)
                score += 5;
            return score;
        }

        private static string Pick(Dictionary<int, NamePick> best, int nameId)
        {
            NamePick pick;
            return best.TryGetValue(nameId, out pick) ? pick.Text : string.Empty;
        }

        private double GetAdvanceWidthForGlyph(int glyphId)
        {
            if (glyphId < 0 || glyphId >= _advanceWidths.Length)
                return 0;
            return _advanceWidths[glyphId];
        }

        private TrueTypeGlyphOutline GetOutlineForGlyph(int glyphId)
        {
            if (glyphId < 0 || glyphId >= GlyphCount)
                return EmptyOutline(0);

            lock (_glyphLock)
            {
                if (_glyphs[glyphId] != null)
                    return _glyphs[glyphId];

                var outline = DecodeGlyph(glyphId, 0);
                if (outline == null)
                    outline = EmptyOutline(GetAdvanceWidthForGlyph(glyphId));
                outline.AdvanceWidth = GetAdvanceWidthForGlyph(glyphId);
                _glyphs[glyphId] = outline;
                return outline;
            }
        }

        private TrueTypeGlyphOutline DecodeGlyph(int glyphId, int depth)
        {
            if (depth > MaxCompositeDepth || glyphId < 0 || glyphId >= GlyphCount)
                return EmptyOutline(0);

            int start = _loca[glyphId];
            int end = _loca[glyphId + 1];
            int length = end - start;
            if (length <= 0)
                return EmptyOutline(GetAdvanceWidthForGlyph(glyphId));

            TrueTypeTableRecord glyf;
            if (!_directory.TryGet(TrueTypeBinary.Tag("glyf"), out glyf))
                return EmptyOutline(0);
            if (start < 0 || length < 2 || start + length > glyf.Length)
                return EmptyOutline(0);

            int offset = glyf.Offset + start;
            int contourCount = _reader.ReadInt16At(offset);
            if (contourCount > 0)
                return DecodeSimpleGlyph(offset, contourCount);
            if (contourCount < 0)
                return DecodeCompositeGlyph(offset, depth);
            return EmptyOutline(GetAdvanceWidthForGlyph(glyphId));
        }

        private TrueTypeGlyphOutline DecodeSimpleGlyph(int offset, int contourCount)
        {
            int endPtsOffset = offset + 10;
            if (endPtsOffset + 2 * contourCount + 2 > _reader.Length)
                return EmptyOutline(0);

            var endPts = new int[contourCount];
            int last = -1;
            for (int i = 0; i < contourCount; i++)
            {
                endPts[i] = _reader.ReadUInt16At(endPtsOffset + 2 * i);
                if (endPts[i] < last)
                    return EmptyOutline(0);
                last = endPts[i];
            }

            int pointCount = endPts[contourCount - 1] + 1;
            if (pointCount <= 0 || pointCount > 10000)
                return EmptyOutline(0);

            int instructionLength = _reader.ReadUInt16At(endPtsOffset + 2 * contourCount);
            int flagsOffset = endPtsOffset + 2 * contourCount + 2 + instructionLength;
            if (flagsOffset > _reader.Length)
                return EmptyOutline(0);

            var flags = new byte[pointCount];
            int pos = flagsOffset;
            for (int i = 0; i < pointCount; i++)
            {
                if (pos >= _reader.Length)
                    return EmptyOutline(0);
                byte flag = _reader.Data[pos++];
                flags[i] = flag;
                if ((flag & RepeatFlag) != 0)
                {
                    if (pos >= _reader.Length)
                        return EmptyOutline(0);
                    int repeat = _reader.Data[pos++];
                    for (int r = 0; r < repeat; r++)
                    {
                        i++;
                        if (i >= pointCount)
                            return EmptyOutline(0);
                        flags[i] = flag;
                    }
                }
            }

            var xs = new int[pointCount];
            int x = 0;
            for (int i = 0; i < pointCount; i++)
            {
                byte flag = flags[i];
                if ((flag & XShortVector) != 0)
                {
                    if (pos >= _reader.Length)
                        return EmptyOutline(0);
                    int dx = _reader.Data[pos++];
                    x += (flag & XIsSameOrPositive) != 0 ? dx : -dx;
                }
                else if ((flag & XIsSameOrPositive) == 0)
                {
                    if (pos + 2 > _reader.Length)
                        return EmptyOutline(0);
                    x += (short)((_reader.Data[pos] << 8) | _reader.Data[pos + 1]);
                    pos += 2;
                }

                xs[i] = x;
            }

            var ys = new int[pointCount];
            int y = 0;
            for (int i = 0; i < pointCount; i++)
            {
                byte flag = flags[i];
                if ((flag & YShortVector) != 0)
                {
                    if (pos >= _reader.Length)
                        return EmptyOutline(0);
                    int dy = _reader.Data[pos++];
                    y += (flag & YIsSameOrPositive) != 0 ? dy : -dy;
                }
                else if ((flag & YIsSameOrPositive) == 0)
                {
                    if (pos + 2 > _reader.Length)
                        return EmptyOutline(0);
                    y += (short)((_reader.Data[pos] << 8) | _reader.Data[pos + 1]);
                    pos += 2;
                }

                ys[i] = y;
            }

            var outline = new TrueTypeGlyphOutline();
            int start = 0;
            for (int c = 0; c < contourCount; c++)
            {
                int end = endPts[c];
                var contour = new List<TrueTypeGlyphPoint>(end - start + 1);
                for (int i = start; i <= end; i++)
                    contour.Add(new TrueTypeGlyphPoint(xs[i], ys[i], (flags[i] & OnCurvePoint) != 0));
                if (contour.Count >= 2)
                    outline.Contours.Add(contour);
                start = end + 1;
            }

            return outline;
        }

        private TrueTypeGlyphOutline DecodeCompositeGlyph(int offset, int depth)
        {
            int pos = offset + 10;
            var result = new TrueTypeGlyphOutline();

            while (true)
            {
                if (pos + 4 > _reader.Length)
                    break;

                int flags = (_reader.Data[pos] << 8) | _reader.Data[pos + 1];
                int glyphIndex = (_reader.Data[pos + 2] << 8) | _reader.Data[pos + 3];
                pos += 4;

                int arg1;
                int arg2;
                if ((flags & Arg1And2AreWords) != 0)
                {
                    if (pos + 4 > _reader.Length)
                        break;
                    arg1 = (short)((_reader.Data[pos] << 8) | _reader.Data[pos + 1]);
                    arg2 = (short)((_reader.Data[pos + 2] << 8) | _reader.Data[pos + 3]);
                    pos += 4;
                }
                else
                {
                    if (pos + 2 > _reader.Length)
                        break;
                    arg1 = (sbyte)_reader.Data[pos];
                    arg2 = (sbyte)_reader.Data[pos + 1];
                    pos += 2;
                }

                double m00 = 1, m01 = 0, m10 = 0, m11 = 1;
                if ((flags & WeHaveAScale) != 0)
                {
                    if (pos + 2 > _reader.Length)
                        break;
                    m00 = m11 = TrueTypeBinary.F2Dot14((short)((_reader.Data[pos] << 8) | _reader.Data[pos + 1]));
                    pos += 2;
                }
                else if ((flags & WeHaveAnXAndYScale) != 0)
                {
                    if (pos + 4 > _reader.Length)
                        break;
                    m00 = TrueTypeBinary.F2Dot14((short)((_reader.Data[pos] << 8) | _reader.Data[pos + 1]));
                    m11 = TrueTypeBinary.F2Dot14((short)((_reader.Data[pos + 2] << 8) | _reader.Data[pos + 3]));
                    pos += 4;
                }
                else if ((flags & WeHaveATwoByTwo) != 0)
                {
                    if (pos + 8 > _reader.Length)
                        break;
                    m00 = TrueTypeBinary.F2Dot14((short)((_reader.Data[pos] << 8) | _reader.Data[pos + 1]));
                    m01 = TrueTypeBinary.F2Dot14((short)((_reader.Data[pos + 2] << 8) | _reader.Data[pos + 3]));
                    m10 = TrueTypeBinary.F2Dot14((short)((_reader.Data[pos + 4] << 8) | _reader.Data[pos + 5]));
                    m11 = TrueTypeBinary.F2Dot14((short)((_reader.Data[pos + 6] << 8) | _reader.Data[pos + 7]));
                    pos += 8;
                }

                var component = DecodeGlyph(glyphIndex, depth + 1);
                if (component != null && component.Contours.Count > 0)
                {
                    double tx = 0;
                    double ty = 0;
                    bool xyValues = (flags & ArgsAreXyValues) != 0;
                    if (xyValues)
                    {
                        tx = arg1;
                        ty = arg2;
                        if ((flags & ScaledComponentOffset) != 0)
                        {
                            double ntx = m00 * tx + m01 * ty;
                            double nty = m10 * tx + m11 * ty;
                            tx = ntx;
                            ty = nty;
                        }
                    }

                    var transformed = TransformOutline(component, m00, m01, m10, m11, tx, ty);
                    if (!xyValues)
                        AlignComponent(result, transformed, arg1, arg2);

                    foreach (var contour in transformed.Contours)
                        result.Contours.Add(contour);
                }

                if ((flags & MoreComponents) == 0)
                {
                    if ((flags & WeHaveInstructions) != 0 && pos + 2 <= _reader.Length)
                    {
                        int instructionLength = (_reader.Data[pos] << 8) | _reader.Data[pos + 1];
                        pos += 2 + Math.Max(0, instructionLength);
                    }

                    break;
                }
            }

            return result;
        }

        private static TrueTypeGlyphOutline TransformOutline(
            TrueTypeGlyphOutline source,
            double m00, double m01, double m10, double m11,
            double tx, double ty)
        {
            var result = new TrueTypeGlyphOutline();
            foreach (var contour in source.Contours)
            {
                var copy = new List<TrueTypeGlyphPoint>(contour.Count);
                foreach (var p in contour)
                {
                    copy.Add(new TrueTypeGlyphPoint(
                        m00 * p.X + m01 * p.Y + tx,
                        m10 * p.X + m11 * p.Y + ty,
                        p.OnCurve));
                }

                result.Contours.Add(copy);
            }

            return result;
        }

        private static void AlignComponent(TrueTypeGlyphOutline parent, TrueTypeGlyphOutline component, int parentPointIndex, int componentPointIndex)
        {
            var parentPoints = parent.AllPoints;
            var componentPoints = component.AllPoints;
            if (parentPointIndex < 0 || parentPointIndex >= parentPoints.Count)
                return;
            if (componentPointIndex < 0 || componentPointIndex >= componentPoints.Count)
                return;

            var from = componentPoints[componentPointIndex];
            var to = parentPoints[parentPointIndex];
            double dx = to.X - from.X;
            double dy = to.Y - from.Y;
            foreach (var contour in component.Contours)
            {
                foreach (var p in contour)
                {
                    p.X += dx;
                    p.Y += dy;
                }
            }
        }

        private static List<Curve2D> ContourToCurves(List<TrueTypeGlyphPoint> contour, Vec2D origin, double scale)
        {
            var expanded = ExpandImpliedOnCurve(contour);
            var curves = new List<Curve2D>();
            if (expanded.Count < 2)
                return curves;

            int count = expanded.Count;
            int start;
            if (expanded[0].OnCurve)
                start = 0;
            else if (expanded[count - 1].OnCurve)
                start = count - 1;
            else
                throw new InvalidDataException("TrueType contour has no valid on-curve start point.");

            Vec2D current = MapPoint(expanded[start], origin, scale);
            Vec2D first = current;
            int i = start;
            do
            {
                int nextIndex = (i + 1) % count;
                var next = expanded[nextIndex];
                if (next.OnCurve)
                {
                    Vec2D end = MapPoint(next, origin, scale);
                    AddLine(curves, current, end);
                    current = end;
                    i = nextIndex;
                }
                else
                {
                    int endIndex = (nextIndex + 1) % count;
                    var endPoint = expanded[endIndex];
                    if (!endPoint.OnCurve)
                        throw new InvalidDataException("TrueType contour contains consecutive off-curve points after expansion.");

                    Vec2D control = MapPoint(next, origin, scale);
                    Vec2D end = MapPoint(endPoint, origin, scale);
                    AddQuadratic(curves, current, control, end);
                    current = end;
                    i = endIndex;
                }
            }
            while (i != start);

            AddLine(curves, current, first);
            return curves;
        }

        private static List<TrueTypeGlyphPoint> ExpandImpliedOnCurve(List<TrueTypeGlyphPoint> contour)
        {
            var expanded = new List<TrueTypeGlyphPoint>(contour.Count * 2);
            int n = contour.Count;
            for (int i = 0; i < n; i++)
            {
                var current = contour[i];
                expanded.Add(current);
                var next = contour[(i + 1) % n];
                if (!current.OnCurve && !next.OnCurve)
                {
                    expanded.Add(new TrueTypeGlyphPoint(
                        (current.X + next.X) * 0.5,
                        (current.Y + next.Y) * 0.5,
                        true));
                }
            }

            return expanded;
        }

        private static Vec2D MapPoint(TrueTypeGlyphPoint point, Vec2D origin, double scale)
        {
            return new Vec2D(origin.X + point.X * scale, origin.Y + point.Y * scale);
        }

        private static void AddLine(List<Curve2D> curves, Vec2D start, Vec2D end)
        {
            if ((end - start).LengthSquared() <= 1e-18)
                return;
            curves.Add(new Line2D(start, end));
        }

        private static void AddQuadratic(List<Curve2D> curves, Vec2D p0, Vec2D q, Vec2D p2)
        {
            if ((p2 - p0).LengthSquared() <= 1e-18 && (q - p0).LengthSquared() <= 1e-18)
                return;

            Vec2D c1 = (p0 + 2.0 * q) / 3.0;
            Vec2D c2 = (p2 + 2.0 * q) / 3.0;
            curves.Add(new Bezier2D(p0, c1, c2, p2));
        }

        private TrueTypeGlyphOutline EmptyOutline(double advance)
        {
            return new TrueTypeGlyphOutline { AdvanceWidth = advance };
        }

        private static bool ContainsWord(string text, string word)
        {
            if (string.IsNullOrEmpty(text) || string.IsNullOrEmpty(word))
                return false;
            return text.IndexOf(word, StringComparison.OrdinalIgnoreCase) >= 0;
        }

        private sealed class NamePick
        {
            public int Score;
            public string Text;
        }
    }

    internal sealed class TrueTypeCmap
    {
        private readonly Func<int, int> _lookup;

        private TrueTypeCmap(Func<int, int> lookup)
        {
            _lookup = lookup;
        }

        public int GetGlyphId(int code)
        {
            int glyphId = _lookup(code);
            return glyphId < 0 ? 0 : glyphId;
        }

        public static TrueTypeCmap TryRead(TrueTypeBinary reader, int offset, int format)
        {
            if (format == 4)
                return ReadFormat4(reader, offset);
            if (format == 12)
                return ReadFormat12(reader, offset);
            if (format == 6)
                return ReadFormat6(reader, offset);
            if (format == 0)
                return ReadFormat0(reader, offset);
            return null;
        }

        private static TrueTypeCmap ReadFormat4(TrueTypeBinary reader, int offset)
        {
            if (offset + 14 > reader.Length)
                return null;

            int length = reader.ReadUInt16At(offset + 2);
            if (length < 16 || offset + length > reader.Length)
                return null;

            int segCount = reader.ReadUInt16At(offset + 6) / 2;
            if (segCount <= 0 || segCount > 16384)
                return null;

            int endCodeOffset = offset + 14;
            int startCodeOffset = endCodeOffset + 2 * segCount + 2;
            int idDeltaOffset = startCodeOffset + 2 * segCount;
            int idRangeOffsetOffset = idDeltaOffset + 2 * segCount;
            if (idRangeOffsetOffset + 2 * segCount > offset + length)
                return null;

            var startCode = new int[segCount];
            var endCode = new int[segCount];
            var idDelta = new int[segCount];
            var idRangeOffset = new int[segCount];
            for (int i = 0; i < segCount; i++)
            {
                startCode[i] = reader.ReadUInt16At(startCodeOffset + 2 * i);
                endCode[i] = reader.ReadUInt16At(endCodeOffset + 2 * i);
                idDelta[i] = reader.ReadInt16At(idDeltaOffset + 2 * i);
                idRangeOffset[i] = reader.ReadUInt16At(idRangeOffsetOffset + 2 * i);
            }

            return new TrueTypeCmap(code =>
            {
                if (code < 0 || code > 0xFFFF)
                    return 0;

                for (int i = 0; i < segCount; i++)
                {
                    if (code < startCode[i] || code > endCode[i])
                        continue;

                    int glyphId;
                    if (idRangeOffset[i] == 0)
                    {
                        glyphId = (code + idDelta[i]) & 0xFFFF;
                    }
                    else
                    {
                        int glyphOffset = idRangeOffsetOffset + 2 * i + idRangeOffset[i] + 2 * (code - startCode[i]);
                        if (glyphOffset + 2 > offset + length)
                            return 0;
                        glyphId = reader.ReadUInt16At(glyphOffset);
                        if (glyphId != 0)
                            glyphId = (glyphId + idDelta[i]) & 0xFFFF;
                    }

                    return glyphId;
                }

                return 0;
            });
        }

        private static TrueTypeCmap ReadFormat12(TrueTypeBinary reader, int offset)
        {
            if (offset + 16 > reader.Length)
                return null;

            uint length = reader.ReadUInt32At(offset + 4);
            uint numGroups = reader.ReadUInt32At(offset + 12);
            if (numGroups > 100000 || offset + 16 + 12 * numGroups > reader.Length)
                return null;
            if (length < 16 + 12 * numGroups)
                return null;

            var starts = new uint[numGroups];
            var ends = new uint[numGroups];
            var glyphs = new uint[numGroups];
            for (uint g = 0; g < numGroups; g++)
            {
                int rec = offset + 16 + (int)g * 12;
                starts[g] = reader.ReadUInt32At(rec);
                ends[g] = reader.ReadUInt32At(rec + 4);
                glyphs[g] = reader.ReadUInt32At(rec + 8);
            }

            return new TrueTypeCmap(code =>
            {
                if (code < 0)
                    return 0;

                int lo = 0;
                int hi = starts.Length - 1;
                while (lo <= hi)
                {
                    int mid = lo + (hi - lo) / 2;
                    if ((uint)code < starts[mid])
                        hi = mid - 1;
                    else if ((uint)code > ends[mid])
                        lo = mid + 1;
                    else
                    {
                        uint glyphId = glyphs[mid] + ((uint)code - starts[mid]);
                        return glyphId > 0xFFFF ? 0 : (int)glyphId;
                    }
                }

                return 0;
            });
        }

        private static TrueTypeCmap ReadFormat6(TrueTypeBinary reader, int offset)
        {
            if (offset + 10 > reader.Length)
                return null;

            int firstCode = reader.ReadUInt16At(offset + 6);
            int entryCount = reader.ReadUInt16At(offset + 8);
            if (offset + 10 + 2 * entryCount > reader.Length)
                return null;

            var ids = new int[entryCount];
            for (int i = 0; i < entryCount; i++)
                ids[i] = reader.ReadUInt16At(offset + 10 + 2 * i);

            return new TrueTypeCmap(code =>
            {
                int index = code - firstCode;
                if (index < 0 || index >= ids.Length)
                    return 0;
                return ids[index];
            });
        }

        private static TrueTypeCmap ReadFormat0(TrueTypeBinary reader, int offset)
        {
            if (offset + 6 + 256 > reader.Length)
                return null;

            var ids = new int[256];
            for (int i = 0; i < 256; i++)
                ids[i] = reader.Data[offset + 6 + i];

            return new TrueTypeCmap(code =>
            {
                if (code < 0 || code > 255)
                    return 0;
                return ids[code];
            });
        }
    }
}
