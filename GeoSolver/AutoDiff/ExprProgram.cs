using System.Runtime.CompilerServices;

namespace GeoSolver
{
    /// <summary>
    /// AOT-safe postfix bytecode for <see cref="Expr"/>. No IL emit, no source generators.
    /// </summary>
    public sealed class ExprProgram
    {
        public const int OpLoadConst = 0;
        public const int OpLoadParam = 1;
        public const int OpAdd = 2;
        public const int OpSub = 3;
        public const int OpMul = 4;
        public const int OpDiv = 5;
        public const int OpNeg = 6;
        public const int OpSqrt = 7;
        public const int OpPow2 = 8;
        public const int OpAbs = 9;
        public const int OpSign = 10;
        public const int OpSin = 11;
        public const int OpCos = 12;
        public const int OpASin = 13;
        public const int OpACos = 14;
        public const int OpAddC = 15;
        public const int OpMulC = 16;
        public const int OpSubSq = 17;
        public const int OpLoadPSq = 18;
        public const int OpMulPP = 19;
        public const int OpSubPP = 20;
        public const int OpSubSqPP = 21;
        public const int OpCosP = 22;
        public const int OpSinMulPP = 23;
        public const int OpSumSqPP = 24;
        public const int OpAddSubSqPP = 25;
        public const int OpAddMulPP = 26;
        public const int OpMulSinCosP = 27;
        public const int OpSqrtSumSqC = 28;
        public const int OpDivPSqC = 29;

        private readonly int[] _code;
        private readonly double[] _consts;
        private readonly Param[] _params;
        private readonly int _stackDepth;

        public int InstructionCount { get { return _code.Length; } }

        private ExprProgram(int[] code, double[] consts, Param[] paramSlots, int stackDepth)
        {
            _code = code;
            _consts = consts;
            _params = paramSlots;
            _stackDepth = stackDepth < 1 ? 1 : stackDepth;
        }

        public static ExprProgram From(Expr expr)
        {
            if (expr == null)
                throw new ArgumentNullException(nameof(expr));

            var c = new Compiler();
            c.Emit(expr);
            return c.Build();
        }

        public Func<double> AsFunc()
        {
            return Evaluate;
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public unsafe double Evaluate()
        {
            int n = _code.Length;
            if (n == 0)
                return 0;

            Param[] p = _params;
            double* stack0 = stackalloc double[_stackDepth];
            fixed (int* code0 = _code)
            fixed (double* k0 = _consts)
            {
                int* ip = code0;
                int* end = code0 + n;
                double* k = k0;
                double* sp = stack0 - 1;
                while (ip < end)
                {
                    int inst = *ip++;
                    int op = inst & 255;
                    int arg = inst >> 8;
                    switch (op)
                    {
                        case OpLoadConst:
                            *++sp = k[arg];
                            break;
                        case OpLoadParam:
                            *++sp = p[arg].Value;
                            break;
                        case OpAdd:
                            {
                                double b = *sp;
                                --sp;
                                *sp += b;
                                break;
                            }
                        case OpSub:
                            {
                                double b = *sp;
                                --sp;
                                *sp -= b;
                                break;
                            }
                        case OpMul:
                            {
                                double b = *sp;
                                --sp;
                                *sp *= b;
                                break;
                            }
                        case OpDiv:
                            {
                                double b = *sp;
                                --sp;
                                *sp /= b;
                                break;
                            }
                        case OpNeg:
                            *sp = -*sp;
                            break;
                        case OpSqrt:
                            {
                                double a = *sp;
                                if (a < 0)
                                    throw new Exception();
                                *sp = Math.Sqrt(a);
                                break;
                            }
                        case OpPow2:
                            {
                                double a = *sp;
                                *sp = a * a;
                                break;
                            }
                        case OpAbs:
                            *sp = Math.Abs(*sp);
                            break;
                        case OpSign:
                            *sp = Math.Sign(*sp);
                            break;
                        case OpSin:
                            *sp = Math.Sin(*sp);
                            break;
                        case OpCos:
                            *sp = Math.Cos(*sp);
                            break;
                        case OpASin:
                            {
                                double a = *sp;
                                if (a > 1 || a < -1)
                                    throw new Exception();
                                *sp = Math.Asin(a);
                                break;
                            }
                        case OpACos:
                            {
                                double a = *sp;
                                if (a > 1 || a < -1)
                                    throw new Exception();
                                *sp = Math.Acos(a);
                                break;
                            }
                        case OpAddC:
                            *sp += k[arg];
                            break;
                        case OpMulC:
                            *sp *= k[arg];
                            break;
                        case OpSubSq:
                            {
                                double b = *sp;
                                --sp;
                                double d = *sp - b;
                                *sp = d * d;
                                break;
                            }
                        case OpLoadPSq:
                            {
                                double a = p[arg].Value;
                                *++sp = a * a;
                                break;
                            }
                        case OpMulPP:
                            *++sp = p[arg & 255].Value * p[arg >> 8].Value;
                            break;
                        case OpSubPP:
                            *++sp = p[arg & 255].Value - p[arg >> 8].Value;
                            break;
                        case OpSubSqPP:
                            {
                                double d = p[arg & 255].Value - p[arg >> 8].Value;
                                *++sp = d * d;
                                break;
                            }
                        case OpCosP:
                            *++sp = Math.Cos(p[arg].Value);
                            break;
                        case OpSinMulPP:
                            *++sp = Math.Sin(p[arg & 255].Value * p[arg >> 8].Value);
                            break;
                        case OpSumSqPP:
                            {
                                double a = p[arg & 255].Value;
                                double b = p[arg >> 8].Value;
                                *++sp = a * a + b * b;
                                break;
                            }
                        case OpAddSubSqPP:
                            {
                                double d0 = p[arg & 63].Value - p[(arg >> 6) & 63].Value;
                                double d1 = p[(arg >> 12) & 63].Value - p[(arg >> 18) & 63].Value;
                                *++sp = d0 * d0 + d1 * d1;
                                break;
                            }
                        case OpAddMulPP:
                            *++sp = p[arg & 63].Value * p[(arg >> 6) & 63].Value
                                  + p[(arg >> 12) & 63].Value * p[(arg >> 18) & 63].Value;
                            break;
                        case OpMulSinCosP:
                            *++sp = Math.Sin(p[arg & 255].Value * p[(arg >> 8) & 255].Value)
                                  * Math.Cos(p[arg >> 16].Value);
                            break;
                        case OpSqrtSumSqC:
                            {
                                double a = p[arg & 255].Value;
                                double b = p[(arg >> 8) & 255].Value;
                                double s = a * a + b * b + k[arg >> 16];
                                if (s < 0)
                                    throw new Exception();
                                *++sp = Math.Sqrt(s);
                                break;
                            }
                        case OpDivPSqC:
                            {
                                double a = p[arg & 255].Value;
                                *sp /= a * a + k[arg >> 8];
                                break;
                            }
                        default:
                            throw new InvalidOperationException("Unknown ExprProgram opcode " + op);
                    }
                }

                return stack0[0];
            }
        }

        private sealed class Compiler
        {
            private readonly List<int> _code = new List<int>();
            private readonly List<double> _consts = new List<double>();
            private readonly List<Param> _params = new List<Param>();
            private readonly Dictionary<Param, int> _paramIndex = new Dictionary<Param, int>();
            private int _depth;
            private int _maxDepth;

            public void Emit(Expr e)
            {
                switch (e.Type)
                {
                    case ExprType.Constant:
                        PushUnary(OpLoadConst, AddConst(e.Value.Value));
                        return;
                    case ExprType.Parameter:
                        PushUnary(OpLoadParam, AddParam(e.Value));
                        return;
                    case ExprType.Function:
                    case ExprType.Differentiate:
                        PushUnary(OpLoadConst, AddConst(double.NaN));
                        return;
                    case ExprType.Add:
                        if (e.B.Type == ExprType.Constant)
                        {
                            Emit(e.A);
                            EmitArg(OpAddC, AddConst(e.B.Value.Value));
                            return;
                        }
                        if (e.A.Type == ExprType.Constant)
                        {
                            Emit(e.B);
                            EmitArg(OpAddC, AddConst(e.A.Value.Value));
                            return;
                        }
                        if (TryParamSquare(e.A, out int sqA) && TryParamSquare(e.B, out int sqB))
                        {
                            PushUnary(OpSumSqPP, Pack(sqA, sqB));
                            return;
                        }
                        if (TryPow2SubPP(e.A, out int d0i, out int d0j) && TryPow2SubPP(e.B, out int d1i, out int d1j)
                            && Fits6(d0i, d0j, d1i, d1j))
                        {
                            PushUnary(OpAddSubSqPP, Pack4(d0i, d0j, d1i, d1j));
                            return;
                        }
                        if (TryMulPP(e.A, out int m0i, out int m0j) && TryMulPP(e.B, out int m1i, out int m1j)
                            && Fits6(m0i, m0j, m1i, m1j))
                        {
                            PushUnary(OpAddMulPP, Pack4(m0i, m0j, m1i, m1j));
                            return;
                        }
                        EmitBinary(e, OpAdd);
                        return;
                    case ExprType.Subtract:
                        if (TryPackedParams(e.A, e.B, out int subI, out int subJ))
                        {
                            PushUnary(OpSubPP, Pack(subI, subJ));
                            return;
                        }
                        EmitBinary(e, OpSub);
                        return;
                    case ExprType.Multiply:
                        if (e.B.Type == ExprType.Constant)
                        {
                            Emit(e.A);
                            EmitArg(OpMulC, AddConst(e.B.Value.Value));
                            return;
                        }
                        if (e.A.Type == ExprType.Constant)
                        {
                            Emit(e.B);
                            EmitArg(OpMulC, AddConst(e.A.Value.Value));
                            return;
                        }
                        if (TryPackedParams(e.A, e.B, out int mulI, out int mulJ))
                        {
                            if (mulI == mulJ)
                                PushUnary(OpLoadPSq, mulI);
                            else
                                PushUnary(OpMulPP, Pack(mulI, mulJ));
                            return;
                        }
                        if (TrySinMulPP(e.A, out int smI, out int smJ) && TryCosP(e.B, out int cP))
                        {
                            PushUnary(OpMulSinCosP, Pack3(smI, smJ, cP));
                            return;
                        }
                        if (TrySinMulPP(e.B, out smI, out smJ) && TryCosP(e.A, out cP))
                        {
                            PushUnary(OpMulSinCosP, Pack3(smI, smJ, cP));
                            return;
                        }
                        EmitBinary(e, OpMul);
                        return;
                    case ExprType.Divide:
                        if (TryPSqPlusConst(e.B, out int denP, out int denC))
                        {
                            Emit(e.A);
                            EmitArg(OpDivPSqC, Pack(denP, denC));
                            return;
                        }
                        EmitBinary(e, OpDiv);
                        return;
                    case ExprType.Negate:
                        EmitUnary(e, OpNeg);
                        return;
                    case ExprType.Sqrt:
                        if (TrySumSqPlusConst(e.A, out int ssI, out int ssJ, out int ssC))
                        {
                            PushUnary(OpSqrtSumSqC, Pack3(ssI, ssJ, ssC));
                            return;
                        }
                        EmitUnary(e, OpSqrt);
                        return;
                    case ExprType.Pow2:
                        if (e.A.Type == ExprType.Subtract && TryPackedParams(e.A.A, e.A.B, out int sqI, out int sqJ))
                        {
                            PushUnary(OpSubSqPP, Pack(sqI, sqJ));
                            return;
                        }
                        if (e.A.Type == ExprType.Subtract)
                        {
                            Emit(e.A.A);
                            Emit(e.A.B);
                            _code.Add(OpSubSq);
                            _depth--;
                            return;
                        }
                        if (e.A.Type == ExprType.Parameter)
                        {
                            PushUnary(OpLoadPSq, AddParam(e.A.Value));
                            return;
                        }
                        EmitUnary(e, OpPow2);
                        return;
                    case ExprType.Abs:
                        EmitUnary(e, OpAbs);
                        return;
                    case ExprType.Sign:
                        EmitUnary(e, OpSign);
                        return;
                    case ExprType.Sin:
                        if (e.A.Type == ExprType.Multiply && TryPackedParams(e.A.A, e.A.B, out int sI, out int sJ))
                        {
                            PushUnary(OpSinMulPP, Pack(sI, sJ));
                            return;
                        }
                        EmitUnary(e, OpSin);
                        return;
                    case ExprType.Cos:
                        if (e.A.Type == ExprType.Parameter)
                        {
                            PushUnary(OpCosP, AddParam(e.A.Value));
                            return;
                        }
                        EmitUnary(e, OpCos);
                        return;
                    case ExprType.ASin:
                        EmitUnary(e, OpASin);
                        return;
                    case ExprType.ACos:
                        EmitUnary(e, OpACos);
                        return;
                    default:
                        PushUnary(OpLoadConst, AddConst(0));
                        return;
                }
            }

            public ExprProgram Build()
            {
                return new ExprProgram(
                    _code.ToArray(),
                    _consts.ToArray(),
                    _params.ToArray(),
                    _maxDepth);
            }

            private void EmitBinary(Expr e, int op)
            {
                Emit(e.A);
                Emit(e.B);
                _code.Add(op);
                _depth--;
            }

            private void EmitUnary(Expr e, int op)
            {
                Emit(e.A);
                _code.Add(op);
            }

            private void EmitArg(int op, int arg)
            {
                _code.Add(op | (arg << 8));
            }

            private bool TryParamSquare(Expr e, out int i)
            {
                i = 0;
                if (e.Type == ExprType.Pow2 && e.A != null && e.A.Type == ExprType.Parameter)
                {
                    i = AddParam(e.A.Value);
                    return i <= 255;
                }
                if (e.Type == ExprType.Multiply && e.A != null && e.B != null
                    && e.A.Type == ExprType.Parameter && e.B.Type == ExprType.Parameter
                    && e.A.Value == e.B.Value)
                {
                    i = AddParam(e.A.Value);
                    return i <= 255;
                }
                return false;
            }

            private bool TryPackedParams(Expr a, Expr b, out int i, out int j)
            {
                i = 0;
                j = 0;
                if (a.Type != ExprType.Parameter || b.Type != ExprType.Parameter)
                    return false;
                i = AddParam(a.Value);
                j = AddParam(b.Value);
                if (i > 255 || j > 255)
                    return false;
                return true;
            }

            private bool TryPow2SubPP(Expr e, out int i, out int j)
            {
                i = 0;
                j = 0;
                return e.Type == ExprType.Pow2 && e.A != null && e.A.Type == ExprType.Subtract
                    && TryPackedParams(e.A.A, e.A.B, out i, out j);
            }

            private bool TryMulPP(Expr e, out int i, out int j)
            {
                i = 0;
                j = 0;
                return e.Type == ExprType.Multiply && TryPackedParams(e.A, e.B, out i, out j);
            }

            private bool TrySinMulPP(Expr e, out int i, out int j)
            {
                i = 0;
                j = 0;
                return e.Type == ExprType.Sin && e.A != null && e.A.Type == ExprType.Multiply
                    && TryPackedParams(e.A.A, e.A.B, out i, out j);
            }

            private bool TryCosP(Expr e, out int i)
            {
                i = 0;
                if (e.Type != ExprType.Cos || e.A == null || e.A.Type != ExprType.Parameter)
                    return false;
                i = AddParam(e.A.Value);
                return i <= 255;
            }

            private bool TryPSqPlusConst(Expr e, out int i, out int c)
            {
                i = 0;
                c = 0;
                if (e.Type != ExprType.Add)
                    return false;
                if (e.B.Type == ExprType.Constant && TryParamSquare(e.A, out i))
                {
                    c = AddConst(e.B.Value.Value);
                    return c <= 255;
                }
                if (e.A.Type == ExprType.Constant && TryParamSquare(e.B, out i))
                {
                    c = AddConst(e.A.Value.Value);
                    return c <= 255;
                }
                return false;
            }

            private bool TrySumSqPlusConst(Expr e, out int i, out int j, out int c)
            {
                i = 0;
                j = 0;
                c = 0;
                if (e.Type != ExprType.Add)
                    return false;
                Expr inner = null;
                Expr k = null;
                if (e.B.Type == ExprType.Constant)
                {
                    inner = e.A;
                    k = e.B;
                }
                else if (e.A.Type == ExprType.Constant)
                {
                    inner = e.B;
                    k = e.A;
                }
                if (inner == null || inner.Type != ExprType.Add || k == null)
                    return false;
                if (!TryParamSquare(inner.A, out i) || !TryParamSquare(inner.B, out j))
                    return false;
                c = AddConst(k.Value.Value);
                return c <= 255;
            }

            private static bool Fits6(int a, int b, int c, int d)
            {
                return a <= 63 && b <= 63 && c <= 63 && d <= 63;
            }

            private static int Pack(int i, int j)
            {
                return i | (j << 8);
            }

            private static int Pack3(int a, int b, int c)
            {
                return a | (b << 8) | (c << 16);
            }

            private static int Pack4(int a, int b, int c, int d)
            {
                return a | (b << 6) | (c << 12) | (d << 18);
            }

            private void PushUnary(int op, int arg)
            {
                _code.Add(op | (arg << 8));
                _depth++;
                if (_depth > _maxDepth)
                    _maxDepth = _depth;
            }

            private int AddConst(double value)
            {
                int i = _consts.Count;
                _consts.Add(value);
                return i;
            }

            private int AddParam(Param p)
            {
                int i;
                if (_paramIndex.TryGetValue(p, out i))
                    return i;
                i = _params.Count;
                _params.Add(p);
                _paramIndex.Add(p, i);
                return i;
            }
        }
    }
}
