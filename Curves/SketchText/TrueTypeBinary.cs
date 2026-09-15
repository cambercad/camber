using System.Text;

namespace Curves
{
    internal sealed class TrueTypeBinary
    {
        public readonly byte[] Data;
        public int Position;

        public TrueTypeBinary(byte[] data)
        {
            Data = data ?? Array.Empty<byte>();
            Position = 0;
        }

        public int Length { get { return Data.Length; } }

        public bool CanRead(int byteCount)
        {
            return byteCount >= 0 && Position >= 0 && Position + byteCount <= Data.Length;
        }

        public void Seek(int position)
        {
            Position = position;
        }

        public byte ReadUInt8()
        {
            return Data[Position++];
        }

        public ushort ReadUInt16()
        {
            int b0 = Data[Position];
            int b1 = Data[Position + 1];
            Position += 2;
            return (ushort)((b0 << 8) | b1);
        }

        public short ReadInt16()
        {
            return (short)ReadUInt16();
        }

        public uint ReadUInt32()
        {
            uint b0 = Data[Position];
            uint b1 = Data[Position + 1];
            uint b2 = Data[Position + 2];
            uint b3 = Data[Position + 3];
            Position += 4;
            return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
        }

        public int ReadInt32()
        {
            return (int)ReadUInt32();
        }

        public byte PeekUInt8()
        {
            return Data[Position];
        }

        public ushort ReadUInt16At(int offset)
        {
            return (ushort)((Data[offset] << 8) | Data[offset + 1]);
        }

        public short ReadInt16At(int offset)
        {
            return (short)ReadUInt16At(offset);
        }

        public uint ReadUInt32At(int offset)
        {
            return ((uint)Data[offset] << 24)
                | ((uint)Data[offset + 1] << 16)
                | ((uint)Data[offset + 2] << 8)
                | Data[offset + 3];
        }

        public bool TryReadUInt32At(int offset, out uint value)
        {
            if (offset < 0 || offset + 4 > Data.Length)
            {
                value = 0;
                return false;
            }

            value = ReadUInt32At(offset);
            return true;
        }

        public string ReadTag()
        {
            if (!CanRead(4))
                return string.Empty;

            return Encoding.ASCII.GetString(Data, Position, 4);
        }

        public static uint Tag(string tag)
        {
            if (string.IsNullOrEmpty(tag) || tag.Length < 4)
                return 0;

            return ((uint)(byte)tag[0] << 24)
                | ((uint)(byte)tag[1] << 16)
                | ((uint)(byte)tag[2] << 8)
                | (byte)tag[3];
        }

        public static double F2Dot14(short value)
        {
            return value / 16384.0;
        }
    }

    internal sealed class TrueTypeTableDirectory
    {
        public uint ScalerType;
        public Dictionary<uint, TrueTypeTableRecord> Tables = new Dictionary<uint, TrueTypeTableRecord>();

        public bool TryGet(uint tag, out TrueTypeTableRecord table)
        {
            return Tables.TryGetValue(tag, out table);
        }

        public bool HasGlyf
        {
            get { return Tables.ContainsKey(TrueTypeBinary.Tag("glyf")) && Tables.ContainsKey(TrueTypeBinary.Tag("loca")); }
        }

        public static bool TryReadCollectionOffsets(TrueTypeBinary reader, out List<int> offsets)
        {
            offsets = new List<int>();
            if (reader == null || reader.Length < 12)
                return false;

            uint magic = reader.ReadUInt32At(0);
            if (magic != TrueTypeBinary.Tag("ttcf"))
            {
                offsets.Add(0);
                return true;
            }

            uint version = reader.ReadUInt32At(4);
            if (version != 0x00010000 && version != 0x00020000)
                return false;

            uint numFonts = reader.ReadUInt32At(8);
            if (numFonts == 0 || numFonts > 64 || 12 + 4 * numFonts > (uint)reader.Length)
                return false;

            for (int i = 0; i < (int)numFonts; i++)
            {
                uint offset = reader.ReadUInt32At(12 + 4 * i);
                if (offset < (uint)reader.Length)
                    offsets.Add((int)offset);
            }

            return offsets.Count > 0;
        }

        public static bool TryRead(TrueTypeBinary reader, int offset, out TrueTypeTableDirectory directory)
        {
            directory = null;
            if (reader == null || offset < 0 || offset + 12 > reader.Length)
                return false;

            uint scaler = reader.ReadUInt32At(offset);
            if (scaler != 0x00010000
                && scaler != TrueTypeBinary.Tag("true")
                && scaler != TrueTypeBinary.Tag("typ1")
                && scaler != TrueTypeBinary.Tag("OTTO"))
            {
                return false;
            }

            int numTables = reader.ReadUInt16At(offset + 4);
            if (numTables <= 0 || numTables > 128)
                return false;

            int recordsStart = offset + 12;
            if (recordsStart + numTables * 16 > reader.Length)
                return false;

            directory = new TrueTypeTableDirectory { ScalerType = scaler };
            for (int i = 0; i < numTables; i++)
            {
                int rec = recordsStart + i * 16;
                uint tag = reader.ReadUInt32At(rec);
                uint tableOffset = reader.ReadUInt32At(rec + 8);
                uint length = reader.ReadUInt32At(rec + 12);
                if (tableOffset > (uint)reader.Length || length > (uint)reader.Length - tableOffset)
                    continue;

                directory.Tables[tag] = new TrueTypeTableRecord
                {
                    Tag = tag,
                    Offset = (int)tableOffset,
                    Length = (int)length,
                };
            }

            return directory.Tables.Count > 0;
        }
    }

    internal sealed class TrueTypeTableRecord
    {
        public uint Tag { get; set; }
        public int Offset { get; set; }
        public int Length { get; set; }

        public bool Contains(int relativeOffset, int byteCount)
        {
            return relativeOffset >= 0 && byteCount >= 0 && relativeOffset + byteCount <= Length;
        }
    }

    internal sealed class TrueTypeNameInfo
    {
        public string Family { get; set; }
        public string TypographicFamily { get; set; }
        public string Subfamily { get; set; }
        public string FullName { get; set; }

        public string BestFamily
        {
            get
            {
                if (!string.IsNullOrWhiteSpace(TypographicFamily))
                    return TypographicFamily;
                return Family ?? string.Empty;
            }
        }
    }

    internal sealed class TrueTypeFontRef
    {
        public string Path { get; set; }
        public int FaceIndex { get; set; }
        public int SfntOffset { get; set; }
        public TrueTypeNameInfo Names { get; set; }
        public bool Bold { get; set; }
        public bool Italic { get; set; }
        public bool HasGlyf { get; set; }

        public bool MatchesFamily(string family)
        {
            if (string.IsNullOrWhiteSpace(family) || Names == null)
                return false;

            return NamesEqual(Names.BestFamily, family)
                || NamesEqual(Names.Family, family)
                || NamesEqual(Names.FullName, family);
        }

        public static bool NamesEqual(string a, string b)
        {
            if (string.IsNullOrWhiteSpace(a) || string.IsNullOrWhiteSpace(b))
                return false;
            return string.Equals(a.Trim(), b.Trim(), StringComparison.OrdinalIgnoreCase);
        }
    }

    internal sealed class TrueTypeGlyphPoint
    {
        public double X;
        public double Y;
        public bool OnCurve;

        public TrueTypeGlyphPoint(double x, double y, bool onCurve)
        {
            X = x;
            Y = y;
            OnCurve = onCurve;
        }
    }

    internal sealed class TrueTypeGlyphOutline
    {
        public double AdvanceWidth;
        public List<List<TrueTypeGlyphPoint>> Contours = new List<List<TrueTypeGlyphPoint>>();

        public List<TrueTypeGlyphPoint> AllPoints
        {
            get
            {
                var points = new List<TrueTypeGlyphPoint>();
                foreach (var contour in Contours)
                    points.AddRange(contour);
                return points;
            }
        }
    }
}
