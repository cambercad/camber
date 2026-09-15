namespace Curves
{
    internal sealed class TrueTypeFontCatalog
    {
        private static readonly object SharedLock = new object();
        private static TrueTypeFontCatalog _shared;

        private static readonly string[] FallbackFamilies =
        {
            "Calibri",
            "Arial",
            "Segoe UI",
            "Tahoma",
            "Verdana",
            "Liberation Sans",
            "DejaVu Sans",
            "FreeSans",
            "Noto Sans",
            "Ubuntu",
            "Nimbus Sans L",
            "Nimbus Sans",
            "Helvetica Neue",
            "Helvetica",
        };

        private readonly List<string> _searchPaths = new List<string>();
        private readonly List<string> _explicitFiles = new List<string>();
        private readonly object _sync = new object();
        private List<TrueTypeFontRef> _faces;
        private readonly Dictionary<string, TrueTypeFontFile> _loaded = new Dictionary<string, TrueTypeFontFile>(StringComparer.OrdinalIgnoreCase);

        public static TrueTypeFontCatalog Shared
        {
            get
            {
                lock (SharedLock)
                {
                    if (_shared == null)
                        _shared = FromSystem();
                    return _shared;
                }
            }
        }

        public static TrueTypeFontCatalog FromSystem()
        {
            var catalog = new TrueTypeFontCatalog();
            catalog._searchPaths.AddRange(SystemFontDirectories());
            return catalog;
        }

        public static TrueTypeFontCatalog FromSearchPaths(IReadOnlyList<string> extraSearchPaths)
        {
            var catalog = new TrueTypeFontCatalog();
            catalog._searchPaths.AddRange(SystemFontDirectories());
            if (extraSearchPaths != null)
            {
                foreach (string path in extraSearchPaths)
                {
                    if (!string.IsNullOrWhiteSpace(path))
                        catalog._searchPaths.Add(path);
                }
            }

            return catalog;
        }

        public static TrueTypeFontCatalog FromFontFile(string path)
        {
            var catalog = new TrueTypeFontCatalog();
            if (!string.IsNullOrWhiteSpace(path))
                catalog._explicitFiles.Add(path);
            return catalog;
        }

        public IReadOnlyList<TrueTypeFontRef> Faces
        {
            get
            {
                EnsureScanned();
                return _faces;
            }
        }

        public bool TryResolve(SketchFontSpec font, out TrueTypeFontFile loaded)
        {
            loaded = null;
            var face = ResolveRef(font);
            if (face == null)
                return false;
            return TryLoad(face, out loaded);
        }

        public TrueTypeFontRef ResolveRef(SketchFontSpec font)
        {
            EnsureScanned();
            if (_faces.Count == 0)
                return null;

            bool bold = font != null && (font.Style & SketchFontStyleFlags.Bold) != 0;
            bool italic = font != null && (font.Style & SketchFontStyleFlags.Italic) != 0;
            string family = font != null ? font.Family : null;

            var match = BestMatch(family, bold, italic);
            if (match != null)
                return match;

            foreach (string fallback in FallbackFamilies)
            {
                if (TrueTypeFontRef.NamesEqual(fallback, family))
                    continue;
                match = BestMatch(fallback, bold, italic);
                if (match != null)
                    return match;
            }

            return BestMatch(null, bold, italic) ?? _faces[0];
        }

        public bool TryLoad(TrueTypeFontRef face, out TrueTypeFontFile font)
        {
            font = null;
            if (face == null || string.IsNullOrWhiteSpace(face.Path))
                return false;

            string key = face.Path + "#" + face.FaceIndex;
            lock (_sync)
            {
                if (_loaded.TryGetValue(key, out font))
                    return true;
            }

            byte[] data;
            try
            {
                data = File.ReadAllBytes(face.Path);
            }
            catch
            {
                return false;
            }

            TrueTypeFontFile parsed;
            TrueTypeFontRef ignored;
            if (!TrueTypeFontFile.TryOpen(data, face.FaceIndex, out parsed, out ignored))
                return false;

            lock (_sync)
            {
                _loaded[key] = parsed;
            }

            font = parsed;
            return true;
        }

        private TrueTypeFontRef BestMatch(string family, bool bold, bool italic)
        {
            TrueTypeFontRef best = null;
            int bestScore = int.MinValue;
            foreach (var face in _faces)
            {
                if (!string.IsNullOrWhiteSpace(family) && !face.MatchesFamily(family))
                    continue;

                int score = 0;
                if (face.Bold == bold)
                    score += 4;
                else
                    score -= 1;
                if (face.Italic == italic)
                    score += 4;
                else
                    score -= 2;
                if (!face.Bold && !face.Italic)
                    score += 1;

                if (score > bestScore)
                {
                    bestScore = score;
                    best = face;
                }
            }

            return best;
        }

        private void EnsureScanned()
        {
            lock (_sync)
            {
                if (_faces != null)
                    return;

                var files = new List<string>();
                files.AddRange(_explicitFiles);
                foreach (string dir in _searchPaths)
                    CollectFontFiles(dir, files);

                var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                var unique = new List<string>();
                foreach (string file in files)
                {
                    if (seen.Add(file))
                        unique.Add(file);
                }

                var bag = new System.Collections.Concurrent.ConcurrentBag<TrueTypeFontRef>();
                System.Threading.Tasks.Parallel.ForEach(unique, file =>
                {
                    var local = new List<TrueTypeFontRef>();
                    AddFacesFromFile(file, local);
                    foreach (var face in local)
                        bag.Add(face);
                });

                _faces = bag.ToList();
            }
        }

        private static void CollectFontFiles(string directory, List<string> files)
        {
            if (string.IsNullOrWhiteSpace(directory) || !Directory.Exists(directory))
                return;

            var stack = new Stack<string>();
            stack.Push(directory);
            while (stack.Count > 0)
            {
                string current = stack.Pop();
                try
                {
                    foreach (string file in Directory.EnumerateFiles(current))
                    {
                        if (IsFontFile(file))
                            files.Add(file);
                    }

                    foreach (string child in Directory.EnumerateDirectories(current))
                        stack.Push(child);
                }
                catch (UnauthorizedAccessException)
                {
                }
                catch (IOException)
                {
                }
            }
        }

        private static bool IsFontFile(string path)
        {
            string ext = Path.GetExtension(path);
            return ext.Equals(".ttf", StringComparison.OrdinalIgnoreCase)
                || ext.Equals(".otf", StringComparison.OrdinalIgnoreCase)
                || ext.Equals(".ttc", StringComparison.OrdinalIgnoreCase);
        }

        private static void AddFacesFromFile(string path, List<TrueTypeFontRef> faces)
        {
            FileStream stream;
            try
            {
                stream = File.OpenRead(path);
            }
            catch
            {
                return;
            }

            using (stream)
            {
                byte[] header = ReadRange(stream, 0, 12);
                if (header == null || header.Length < 12)
                    return;

                var headerReader = new TrueTypeBinary(header);
                List<int> offsets;
                if (headerReader.ReadUInt32At(0) == TrueTypeBinary.Tag("ttcf"))
                {
                    uint version = headerReader.ReadUInt32At(4);
                    if (version != 0x00010000 && version != 0x00020000)
                        return;
                    uint numFonts = headerReader.ReadUInt32At(8);
                    if (numFonts == 0 || numFonts > 64)
                        return;
                    byte[] offsetBlock = ReadRange(stream, 12, (int)(4 * numFonts));
                    if (offsetBlock == null || offsetBlock.Length < 4 * numFonts)
                        return;
                    offsets = new List<int>((int)numFonts);
                    var offsetReader = new TrueTypeBinary(offsetBlock);
                    for (int i = 0; i < (int)numFonts; i++)
                        offsets.Add((int)offsetReader.ReadUInt32At(4 * i));
                }
                else
                {
                    offsets = new List<int> { 0 };
                }

                for (int i = 0; i < offsets.Count; i++)
                {
                    TrueTypeFontRef face;
                    if (!TryReadCatalogRef(stream, path, i, offsets[i], out face))
                        continue;
                    faces.Add(face);
                }
            }
        }

        private static bool TryReadCatalogRef(FileStream stream, string path, int faceIndex, int sfntOffset, out TrueTypeFontRef catalogRef)
        {
            catalogRef = null;
            byte[] dirHeader = ReadRange(stream, sfntOffset, 12);
            if (dirHeader == null || dirHeader.Length < 12)
                return false;

            var dirReader = new TrueTypeBinary(dirHeader);
            uint scaler = dirReader.ReadUInt32At(0);
            if (scaler != 0x00010000
                && scaler != TrueTypeBinary.Tag("true")
                && scaler != TrueTypeBinary.Tag("typ1")
                && scaler != TrueTypeBinary.Tag("OTTO"))
            {
                return false;
            }

            int numTables = dirReader.ReadUInt16At(4);
            if (numTables <= 0 || numTables > 128)
                return false;

            byte[] records = ReadRange(stream, sfntOffset + 12, numTables * 16);
            if (records == null || records.Length < numTables * 16)
                return false;

            var recordReader = new TrueTypeBinary(records);
            TrueTypeTableRecord name = null;
            TrueTypeTableRecord head = null;
            TrueTypeTableRecord os2 = null;
            bool hasGlyf = false;
            bool hasLoca = false;
            uint glyfTag = TrueTypeBinary.Tag("glyf");
            uint locaTag = TrueTypeBinary.Tag("loca");
            uint nameTag = TrueTypeBinary.Tag("name");
            uint headTag = TrueTypeBinary.Tag("head");
            uint os2Tag = TrueTypeBinary.Tag("OS/2");

            for (int i = 0; i < numTables; i++)
            {
                int rec = i * 16;
                uint tag = recordReader.ReadUInt32At(rec);
                uint tableOffset = recordReader.ReadUInt32At(rec + 8);
                uint length = recordReader.ReadUInt32At(rec + 12);
                var table = new TrueTypeTableRecord
                {
                    Tag = tag,
                    Offset = (int)tableOffset,
                    Length = (int)length,
                };
                if (tag == glyfTag)
                    hasGlyf = true;
                else if (tag == locaTag)
                    hasLoca = true;
                else if (tag == nameTag)
                    name = table;
                else if (tag == headTag)
                    head = table;
                else if (tag == os2Tag)
                    os2 = table;
            }

            if (!hasGlyf || !hasLoca || head == null || name == null)
                return false;

            byte[] headData = ReadRange(stream, head.Offset, Math.Min(head.Length, 54));
            if (headData == null || headData.Length < 54)
                return false;
            if (new TrueTypeBinary(headData).ReadUInt32At(12) != 0x5F0F3CF5)
                return false;

            int macStyle = new TrueTypeBinary(headData).ReadUInt16At(44);
            var names = ReadNameInfo(ReadRange(stream, name.Offset, name.Length));
            bool bold = false;
            bool italic = false;
            if (os2 != null && os2.Length >= 6)
            {
                byte[] os2Data = ReadRange(stream, os2.Offset, Math.Min(os2.Length, 78));
                if (os2Data != null && os2Data.Length >= 6)
                {
                    var os2Reader = new TrueTypeBinary(os2Data);
                    int weight = os2Reader.ReadUInt16At(4);
                    bold = weight >= 600;
                    if (os2Data.Length >= 64)
                    {
                        int selection = os2Reader.ReadUInt16At(62);
                        bold = bold || (selection & 0x20) != 0;
                        italic = (selection & 0x01) != 0 || (selection & 0x200) != 0;
                    }
                }
            }

            string sub = names != null ? names.Subfamily : string.Empty;
            bold = bold || (macStyle & 0x01) != 0 || ContainsStyleWord(sub, "Bold") || ContainsStyleWord(sub, "Black");
            italic = italic || (macStyle & 0x02) != 0 || ContainsStyleWord(sub, "Italic") || ContainsStyleWord(sub, "Oblique");

            catalogRef = new TrueTypeFontRef
            {
                Path = path,
                FaceIndex = faceIndex,
                SfntOffset = sfntOffset,
                Names = names ?? new TrueTypeNameInfo { Family = "Unknown" },
                Bold = bold,
                Italic = italic,
                HasGlyf = true,
            };
            if (string.IsNullOrWhiteSpace(catalogRef.Names.BestFamily)
                && string.IsNullOrWhiteSpace(catalogRef.Names.FullName))
            {
                catalogRef.Names.Family = "Unknown";
            }
            else if (string.IsNullOrWhiteSpace(catalogRef.Names.Family)
                && !string.IsNullOrWhiteSpace(catalogRef.Names.FullName))
            {
                catalogRef.Names.Family = catalogRef.Names.FullName;
            }

            return true;
        }

        private static TrueTypeNameInfo ReadNameInfo(byte[] table)
        {
            var info = new TrueTypeNameInfo();
            if (table == null || table.Length < 6)
                return info;

            var reader = new TrueTypeBinary(table);
            int format = reader.ReadUInt16At(0);
            int count = reader.ReadUInt16At(2);
            int stringOffset = reader.ReadUInt16At(4);
            if (format > 1 || count < 0 || count > 4096)
                return info;

            var best = new Dictionary<int, int>();
            var texts = new Dictionary<int, string>();
            for (int i = 0; i < count; i++)
            {
                int rec = 6 + i * 12;
                if (rec + 12 > table.Length)
                    break;

                int platform = reader.ReadUInt16At(rec);
                int encoding = reader.ReadUInt16At(rec + 2);
                int language = reader.ReadUInt16At(rec + 4);
                int nameId = reader.ReadUInt16At(rec + 6);
                int length = reader.ReadUInt16At(rec + 8);
                int offset = reader.ReadUInt16At(rec + 10);
                if (nameId != 1 && nameId != 2 && nameId != 4 && nameId != 16 && nameId != 17)
                    continue;
                int abs = stringOffset + offset;
                if (length <= 0 || abs < 0 || abs + length > table.Length)
                    continue;

                string text = DecodeName(table, platform, encoding, abs, length);
                if (string.IsNullOrWhiteSpace(text))
                    continue;

                int score = ScoreName(platform, encoding, language);
                int existing;
                if (!best.TryGetValue(nameId, out existing) || score > existing)
                {
                    best[nameId] = score;
                    texts[nameId] = text.Trim();
                }
            }

            string family;
            string subfamily;
            string full;
            string typo;
            texts.TryGetValue(1, out family);
            texts.TryGetValue(2, out subfamily);
            texts.TryGetValue(4, out full);
            texts.TryGetValue(16, out typo);
            info.Family = family ?? string.Empty;
            info.Subfamily = subfamily ?? string.Empty;
            info.FullName = full ?? string.Empty;
            info.TypographicFamily = typo ?? string.Empty;
            string typoSub;
            if (texts.TryGetValue(17, out typoSub) && !string.IsNullOrWhiteSpace(typoSub))
                info.Subfamily = typoSub;
            return info;
        }

        private static string DecodeName(byte[] table, int platform, int encoding, int offset, int length)
        {
            bool unicode = platform == 0 || platform == 3 || (platform == 2 && encoding == 1);
            if (unicode)
            {
                int charCount = length / 2;
                var chars = new char[charCount];
                for (int i = 0; i < charCount; i++)
                    chars[i] = (char)((table[offset + 2 * i] << 8) | table[offset + 2 * i + 1]);
                return new string(chars);
            }

            var ascii = new char[length];
            for (int i = 0; i < length; i++)
                ascii[i] = (char)table[offset + i];
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

        private static bool ContainsStyleWord(string text, string word)
        {
            if (string.IsNullOrEmpty(text) || string.IsNullOrEmpty(word))
                return false;
            return text.IndexOf(word, StringComparison.OrdinalIgnoreCase) >= 0;
        }

        private static byte[] ReadRange(FileStream stream, int offset, int length)
        {
            if (stream == null || offset < 0 || length <= 0 || offset + length > stream.Length)
                return null;

            var buffer = new byte[length];
            stream.Seek(offset, SeekOrigin.Begin);
            int read = 0;
            while (read < length)
            {
                int n = stream.Read(buffer, read, length - read);
                if (n <= 0)
                    return null;
                read += n;
            }

            return buffer;
        }

        private static List<string> SystemFontDirectories()
        {
            var dirs = new List<string>();
            if (OperatingSystem.IsWindows())
            {
                string windows = Environment.GetFolderPath(Environment.SpecialFolder.Windows);
                if (!string.IsNullOrEmpty(windows))
                    dirs.Add(Path.Combine(windows, "Fonts"));

                string local = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData);
                if (!string.IsNullOrEmpty(local))
                    dirs.Add(Path.Combine(local, "Microsoft", "Windows", "Fonts"));
            }
            else if (OperatingSystem.IsMacOS())
            {
                dirs.Add("/System/Library/Fonts");
                dirs.Add("/Library/Fonts");
                string home = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
                if (!string.IsNullOrEmpty(home))
                    dirs.Add(Path.Combine(home, "Library", "Fonts"));
            }
            else
            {
                dirs.Add("/usr/share/fonts");
                dirs.Add("/usr/local/share/fonts");
                string home = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
                if (!string.IsNullOrEmpty(home))
                {
                    dirs.Add(Path.Combine(home, ".local", "share", "fonts"));
                    dirs.Add(Path.Combine(home, ".fonts"));
                }
            }

            return dirs;
        }
    }
}
