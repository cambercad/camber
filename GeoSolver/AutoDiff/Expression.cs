using System.Text;
using System.Text.RegularExpressions;

namespace GeoSolver
{
    public enum ExprType
    {
        None,
        Parameter, //Variable
        Constant,

        Add,
        Subtract,
        Multiply,
        Divide,
        Sqrt,
        Pow2,
        Abs,
        Sign,
        Negate,
        Sin,
        Cos,
        ASin,
        ACos,

        //The following types can only be used for symbolic manipulation, not the evaluate
        Function, //A general function that depends on other parameters
        Differentiate //Derivative of a general function
    }


    public class FunctionParam : NamedParam
    {
        private HashSet<Param> _dependencies;
        public HashSet<Param> Dependencies { get { return _dependencies; } }

        public FunctionParam(string name, params Param[] dependencies) : base(double.NaN, name)
        {
            _dependencies = new HashSet<Param>();
            for (int i = 0; i < dependencies.Length; ++i)
                _dependencies.Add(dependencies[i]);
        }

        public bool DependsOn(Param p)
        {
            return _dependencies.Contains(p);
        }

        public void Substitute(Param oldParam, Param newParam)
        {
            _dependencies.Remove(oldParam);
            _dependencies.Add(newParam);
        }

        public override string ToString()
        {
            string s = _name + "(";
            int counter = 0;
            foreach (var v in _dependencies)
            {
                if (counter > 0)
                    s += ", ";
                s += v.GetNameOrValue();
                ++counter;
            }
            return s + ")";
        }
    }

    public class NamedParam : Param
    {
        protected string _name;
        public string Name { get { return _name; } }

        public NamedParam(double value, string name) : base(value) { _name = name; }
    }


    //public class VariableNamedParam : VariableParam
    //{
    //    protected string _name;
    //    public string Name { get { return _name; } }

    //    public VariableNamedParam(double value, string name) : base(value) { _name = name; }
    //}


    //public class VariableParam : Param
    //{
    //    public VariableParam(double value) : base(value) { }
    //}

    [Serializable]
    public class Param //: IStorable
    {
        private static int _indexer = 0;

        private int _index = _indexer++; //Mainly for debugging
        private double _value;
        public double Value { get { return _value; } set { _value = value; } }

        /// <summary>
        /// When true, this parameter is treated as a constant during constraint solving.
        /// Frozen parameters cannot be modified by the solver.
        /// </summary>
        public bool Frozen { get; set; }

        /// <summary>Column-scaling hint for the Newton solver (length vs orientation vs contact force).</summary>
        public ParamScaleKind ScaleKind { get; set; }

        public Param(double value) { _value = value; }

        public static Param Zero { get { return new Param(0/*, true*/); } }

        public string GetNameOrValue(string format = "", bool forceValue = false)
        {
            if (!forceValue)
            {
                var n = this as NamedParam;
                if (n != null)
                    return n.Name;
            }
            return _value.ToString(format);
        }

        public override string ToString() { return "[" + _index + "] " + _value.ToString(); }

        public string ToString(Dictionary<Param, string> names)
        {
            return names[this];
        }
    }

    public interface IEnumerableParams : IEnumerable<Param> { }

    public sealed class Expr : IEnumerableParams//, IStorable
    {
        private ExprType _type; public ExprType Type { get { return _type; } set { _type = value; } }
        private Expr _a; public Expr a { get { return _a; } }
        public Expr A { get { return _a; } }
        private Expr _b; public Expr b { get { return _b; } }
        public Expr B { get { return _b; } }
        private Param _value; public Param Value { get { return _value; } }
        //private string _name; public string Name { get { return _name; } set { _name = value; } } 

        //public Expr() { _type = ExprType.Constant; _value = new Param(0); }
        private Expr() { }

        public Expr(ExprType type, Expr a, Expr b, bool enableSimplify = false)
        {
            if(type == ExprType.Parameter || type == ExprType.Constant)
            {
                if (type == null)
                    throw new Exception();
            }

            _type = type;
            _a = a;
            _b = b;

            if(enableSimplify)
            SimplifyInternal();

            //if(type == ExprType.Add || type == ExprType.Multiply)
            //{
            //    if(a.ToString().CompareTo(b.ToString()) > 0)
            //    {
            //        var tmp = _a;
            //        _a = _b;
            //        _b = tmp;
            //    }
            //}
        }

        private struct ExprWithString
        {
            public Expr E;
            public string S;

            public ExprWithString(Expr e) : this()
            {
                E = e;
                S = e.ToString();
            }

            public override string ToString()
            {
                return S;
            }
        }

        private void FuseConst()
        {
            if (Type == ExprType.Multiply)
            {
                List<Expr> children = new List<Expr>();
                _a.CollectOfType(ExprType.Multiply, children);
                _b.CollectOfType(ExprType.Multiply, children);

                List<ExprWithString> c = Convert(children);
                Sort(c);

                //Fuse constants
                FuseConstantsMultiply(c);

                var combined = Combine(c, ExprType.Multiply);
                AssignToThis(combined);
            }

            if (Type == ExprType.Add)
            {
                List<Expr> children = new List<Expr>();
                _a.CollectOfType(ExprType.Add, children);
                _b.CollectOfType(ExprType.Add, children);

                List<ExprWithString> c = Convert(children);
                Sort(c);

                //Fuse constants
                FuseConstantsAdd(c);

                var combined = Combine(c, ExprType.Add);
                AssignToThis(combined);
            }
        }

        private void FuseConstantsAdd(List<ExprWithString> c)
        {
            for (int i = 0; i < c.Count; i++)
            {
                var e = c[i];
                if (e.E.Type != ExprType.Constant)
                    continue;

                int end = i;
                for (int j = i + 1; j < c.Count; j++)
                {
                    if (c[j].E.Type != ExprType.Constant)
                        break;
                    end = j + 1;
                }
                if (end != i)
                {
                    Expr fused = e.E;
                    for (int j = i + 1; j < end; ++j)
                    {
                        fused = fused + c[j].E;
                        fused = fused.FoldConstants();
                    }

                    c.RemoveRange(i, end - i);
                    if (fused.Value.Value != 0)
                        c.Insert(i, new ExprWithString(fused));
                }
            }
        }

        private void FuseConstantsMultiply(List<ExprWithString> c)
        {
            for (int i = 0; i < c.Count; i++)
            {
                var e = c[i];
                if (e.E.Type != ExprType.Constant)
                    continue;

                int end = i;
                for (int j = i + 1; j < c.Count; j++)
                {
                    if (c[j].E.Type != ExprType.Constant)
                        break;
                    end = j + 1;
                }
                if (end != i)
                {
                    Expr fused = e.E;
                    for (int j = i + 1; j < end; ++j)
                    {
                        fused = fused * c[j].E;
                        fused = fused.FoldConstants();
                    }

                    c.RemoveRange(i, end - i);
                    if (fused.Value.Value != 1)
                        c.Insert(i, new ExprWithString(fused));
                }
            }
        }

        private static void SimplifyInternal(Expr e)
        {
            e.SimplifyInternal();
        }
        private void SimplifyInternal()
        {
            AssignToThis(this.FoldConstants());

            if (Type == ExprType.Multiply) //Propagate negate outwards
            {
                bool negateA = _a.Type == ExprType.Negate;
                bool negateB = _b.Type == ExprType.Negate;
                if (negateA && negateB)
                {
                    AssignToThis(_a._a * _b._a);
                }
                else if (negateA)
                {
                    AssignToThis(-(_a._a * _b));
                }
                else if (negateB)
                {
                    AssignToThis(-(_a * _b._a));
                }
            }

            if (Type == ExprType.Add)
            {
                bool negateA = _a.Type == ExprType.Negate;
                bool negateB = _b.Type == ExprType.Negate;
                if (negateA && negateB)
                {
                    AssignToThis(-(_a._a + _b._a));
                }
                else if (negateA)
                {
                    AssignToThis(_b - _a._a);
                }
                else if (negateB)
                {
                    AssignToThis(_a - _b._a);
                }
            }

            if (Type == ExprType.Negate)
            {
                if(_a.Type == ExprType.Negate)
                {
                    AssignToThis(_a._a);
                }
                else if(_a.Type == ExprType.Multiply)//Try to fuse negate into a constant factor
                {
                    var e = _a;

                    List<Expr> children = new List<Expr>();
                    e._a.CollectOfType(ExprType.Multiply, children);
                    e._b.CollectOfType(ExprType.Multiply, children);

                    List<ExprWithString> c = Convert(children);
                    Sort(c);

                    FuseConstantsMultiply(c);

                    if (c[0].E.Type == ExprType.Constant)
                    {
                        c[0] = new ExprWithString(Expr.Constant(-c[0].E.Value.Value));

                        var combined = Combine(c, ExprType.Multiply);
                        AssignToThis(combined);
                    }
                }
            }


            if (Type == ExprType.Multiply)
            {
                if (_b.Type == ExprType.Constant && _a.Type != ExprType.Constant)
                {
                    var tmp = _a;
                    _a = b;
                    _b = tmp;
                }

                List<Expr> children = new List<Expr>();
                _a.CollectOfType(ExprType.Multiply, children);
                _b.CollectOfType(ExprType.Multiply, children);

                List<ExprWithString> c = Convert(children);
                Sort(c);

                //Fuse constants
                FuseConstantsMultiply(c);
                //for (int i = 0; i < c.Count; i++)
                //{
                //    var e = c[i];
                //    if (e.E.Type != ExprType.Constant)
                //        continue;

                //    int end = i;
                //    for (int j = i + 1; j < c.Count; j++)
                //    {
                //        if (c[j].E.Type != ExprType.Constant)
                //            break;
                //        end = j + 1;
                //    }
                //    if (end != i)
                //    {
                //        Expr fused = e.E;
                //        for (int j = i + 1; j < end; ++j)
                //        {
                //            fused = fused * c[j].E;
                //            fused = fused.FoldConstants();
                //        }

                //        c.RemoveRange(i, end - i);
                //        if (fused.Value.Value != 1)
                //            c.Insert(i, new ExprWithString(fused));
                //    }
                //}

                //Fuse identical -> to power
                for (int i = 0; i < c.Count; i++)
                {
                    var e = c[i];
                    int counter = 1;
                    for (int j = i + 1; j < c.Count; j++)
                    {
                        if (c[j].S != e.S)
                            break;
                        ++counter;
                    }
                    if (counter == 2)
                    {
                        c.RemoveRange(i, counter);
                        c.Insert(i, new ExprWithString(Expr.Pow2(e.E)));
                    }
                }

                //if (c.Count == 1)
                //{
                //    Type = c[0].E.Type;
                //    _a = c[0].E._a;
                //    _b = c[0].E._b;
                //}
                //else
                //{
                //    _a = c[0].E;
                //    _b = c[c.Count - 1].E;
                //    for (int i = 1; i < c.Count - 1; i++)
                //        _a *= c[i].E;
                //}
                var combined = Combine(c, ExprType.Multiply);
                AssignToThis(combined);

                if (Type == ExprType.Multiply)
                {
                    TryExpandSumConstantsOnly();

                    if (_a == null || _b == null)
                        throw new Exception();
                }
            }

            if (Type == ExprType.Add)
            {
                if (_b.Type == ExprType.Constant && _a.Type != ExprType.Constant)
                {
                    var tmp = _a;
                    _a = b;
                    _b = tmp;
                }

                List<Expr> children = new List<Expr>();
                _a.CollectOfType(ExprType.Add, children);
                _b.CollectOfType(ExprType.Add, children);

                List<ExprWithString> c = Convert(children);
                Sort(c);

                //Fuse constants
                FuseConstantsAdd(c);
                //for (int i = 0; i < c.Count; i++)
                //{
                //    var e = c[i];
                //    if (e.E.Type != ExprType.Constant)
                //        continue;

                //    int end = i;
                //    for (int j = i + 1; j < c.Count; j++)
                //    {
                //        if (c[j].E.Type != ExprType.Constant)
                //            break;
                //        end = j + 1;
                //    }
                //    if (end != i)
                //    {
                //        Expr fused = e.E;
                //        for (int j = i + 1; j < end; ++j)
                //        {
                //            fused = fused + c[j].E;
                //            fused = fused.FoldConstants();
                //        }

                //        c.RemoveRange(i, end - i);
                //        if (fused.Value.Value != 0)
                //            c.Insert(i, new ExprWithString(fused));
                //    }
                //}

                //Fuse identical -> to muliply
                for (int i = 0; i < c.Count; i++)
                {
                    var e = c[i];
                    int counter = 1;
                    for (int j = i + 1; j < c.Count; j++)
                    {
                        if (c[j].S != e.S)
                            break;
                        ++counter;
                    }
                    if(counter>1)
                    {
                        c.RemoveRange(i, counter);
                        c.Insert(i, new ExprWithString(Expr.Constant(counter) * e.E));
                    }
                }

                //if (c.Count == 1)
                //{
                //    Type = c[0].E.Type;
                //    _a = c[0].E._a;
                //    _b = c[0].E._b;
                //}
                //else
                //{
                //    _a = c[0].E;
                //    _b = c[c.Count - 1].E;
                //    for (int i = 1; i < c.Count-1; i++)
                //        _a += c[i].E;
                //}
                var combined = Combine(c, ExprType.Add);
                AssignToThis(combined);

                if (Type == ExprType.Add)
                {
                    TryFindCommonFactorFront();

                    if (_a == null || _b == null)
                        throw new Exception();
                }
            }

           
            
            //if(Type == ExprType.Multiply || Type == ExprType.Add)
            //{
            //    if (_b.Type == ExprType.Constant && _a.Type != ExprType.Constant)
            //    {
            //        var tmp = _a;
            //        _a = b;
            //        _b = tmp;
            //    }
            //}
        }


        private void AssignToThis(Expr e)
        {
            this._a = e._a;
            this._b = e._b;
            this._type = e._type;
            this._value = e._value;
        }

        private Expr Combine(IList<ExprWithString> list, ExprType type)
        {
            //List<Expr> l = new List<Expr>(list.Count);
            //for (int i = 0; i < list.Count; i++)
            //    l.Add(list[i].E);
            //return Combine(l, type);

            var list2 = new List<Expr>(list.Count);
            for (int i = 0; i < list.Count; i++)
                list2.Add(list[i].E);

            return Combine(list2, type);
        }

        private Expr Combine(IList<Expr> list, ExprType type)
        {
            if(list.Count == 0)
            {
                throw new Exception();
            }

            while(list.Count>1)
            {
                list = FuseAdjacent(list, type);
            }
            return list[0];
        }

        private List<Expr> FuseAdjacent(IList<Expr> list, ExprType type)
        {
            List<Expr> result = new List<Expr>();
            for(int i=0; i<list.Count; i+=2)
            {
                if(i+1>=list.Count)
                {
                    result.Add(list[i]);
                    return result;
                }
                else
                {
                    result.Add(new Expr(type, list[i], list[i+1]));
                }
            }
            return result;
        }

        private void TryExpandSumConstantsOnly()
        {
            if (this.Type != ExprType.Multiply)
                return;
            if (_b.Type == ExprType.Constant && _a.Type != ExprType.Constant)
            {
                var tmp = _a;
                _a = b;
                _b = tmp;
            }
            if (_a.Type != ExprType.Constant)
                return;
            if (_b.Type != ExprType.Add)
                return;

            List<Expr> children = new List<Expr>();
            _b.CollectOfType(ExprType.Add, children);

            if (children.Count < 2)
                throw new Exception();

            var factor = _a;
            List<Expr> newSum = new List<Expr>();
            for(int i=0;i<children.Count; ++i)
            {
                var mul = factor * children[i];
                mul.FuseConst();
                newSum.Add(mul);
            }

            //_a = newSum[0];
            //_b = newSum[1];
            //for (int i = 2; i < newSum.Count; ++i)
            //    _b += newSum[i];
            var combined = Combine(newSum, ExprType.Add);
            AssignToThis(combined);
        }

        private static List<ExprWithString> SetIntersection(List<ExprWithString> a, List<ExprWithString> b)
        {
            List<ExprWithString> intersection = new List<ExprWithString>();
            for(int i=0;i<a.Count;++i)
            {
                if (Contains(b, a[i].S))
                    intersection.Add(a[i]);
            }
            return intersection;
        }

        private static List<ExprWithString> SetDifference(List<ExprWithString> a, List<ExprWithString> subtract)
        {
            List<ExprWithString> sub = new List<ExprWithString>();
            for (int i = 0; i < a.Count; ++i)
            {
                if (!Contains(subtract, a[i].S))
                    sub.Add(a[i]);
            }
            return sub;
        }

            private static bool Contains(List<ExprWithString> b, string s)
        {
            for(int i=0;i< b.Count;++i)
                if (b[i].S == s)
                    return true;
            return false;
        }

        private void TryFindCommonFactorFront()
        {
            if (this.Type != ExprType.Add)
                return;

            List<Expr> children = new List<Expr>();
            _a.CollectOfType(ExprType.Add, children);
            _b.CollectOfType(ExprType.Add, children);

            List<ExprWithString> c = Convert(children);
            Sort(c, children);

            List<List<ExprWithString>> sumOfMul = new List<List<ExprWithString>>();
            for(int i=0;i<children.Count;++i)
            {
                List<Expr> list = new List<Expr>();
                children[i].CollectOfType(ExprType.Multiply, list);

                List<ExprWithString> c2 = Convert(list);
                Sort(c2);

                sumOfMul.Add(c2);
            }

            List<Expr> processed = new List<Expr>();         
            for (int i = 0; i < sumOfMul.Count; ++i)
            {
                List<ExprWithString> set = sumOfMul[i];
                //if(i==sumOfMul.Count-1)
                //{
                //    processed.Add(Combine(set, ExprType.Multiply));
                //    break;
                //}

                for (int j = i + 1; j <= sumOfMul.Count; ++j)
                {
                    var intersection = j == sumOfMul.Count ? new List<ExprWithString>() : SetIntersection(set, sumOfMul[j]);                   

                    if(intersection.Count == 0)
                    {
                        if (j == i + 1)
                        {
                            var cc = Combine(set, ExprType.Multiply);
                            processed.Add(cc);
                        }
                        else
                        {
                            var factor = Combine(set, ExprType.Multiply);
                            List<Expr> remaining = new List<Expr>();
                            for (int k = i; k < j; ++k)
                            {
                                var diff = SetDifference(sumOfMul[k], set);
                                if (diff.Count == 0)
                                    remaining.Add(Expr.Constant(1.0));
                                else
                                    remaining.Add(Combine(diff, ExprType.Multiply));
                            }
                            var rhs = Combine(remaining, ExprType.Add);
                            processed.Add(factor * rhs);
                            i = j - 1;
                        }
                        break;
                    }

                    set = intersection;
                }
            }


            //for (int i=0;i< sumOfMul.Count;++i)
            //{
            //    ExprWithString v = c[i];
            //    if(v.E.Type == ExprType.Multiply)
            //    {
            //        List<Expr> children2 = new List<Expr>();
            //        v.E.CollectOfType(ExprType.Multiply, children2);

            //        //List<ExprWithString> c2 = Convert(children2);
            //        //Sort(c2);

            //        var a = children2[0];
            //        children2.RemoveAt(0);

            //        int end = i;
            //        List<Expr> buffer = new List<Expr>();
            //        buffer.Add(v.E);
            //        for(int j=i+1;j<c.Count;++j)
            //        {
            //            var v2 = c[j];
            //            if (v2.E.Type == ExprType.Multiply)
            //            {
            //                if (a.ToString() == v2.E.a.ToString())
            //                {
            //                    buffer.Add(v2.E.B);
            //                    end = j + 1;
            //                }
            //                else
            //                    break;
            //            }
            //        }
            //        if(end > i)
            //        {
            //            //Expr sum = v.E.B;
            //            //for (int j = i + 1; j < end; ++j)
            //            //{
            //            //    sum += c[j].E.B;
            //            //}
            //            Expr sum = Combine(buffer, ExprType.Add);

            //            c.RemoveRange(i, end - i);
            //            c.Insert(i, new ExprWithString(v.E.A * sum));
            //        }
            //    }
            //}


            //if (c.Count == 1)
            //{
            //    Type = c[0].E.Type;
            //    _a = c[0].E._a;
            //    _b = c[0].E._b;
            //}
            //else
            //{
            //    _a = c[0].E;
            //    _b = c[c.Count - 1].E;
            //    for (int i = 1; i < c.Count - 1; i++)
            //        _a += c[i].E;
            //}

            //List<Expr> combineFirstStage = new List<Expr>();
            //for (int i = 0; i < sumOfMul.Count; ++i)
            //    combineFirstStage.Add(Combine(sumOfMul[i], ExprType.Multiply));

            var combined = Combine(processed, ExprType.Add);
            AssignToThis(combined);
        }


        private List<ExprWithString> Convert(List<Expr> children)
        {
            List<ExprWithString> res = new List<ExprWithString>(children.Count);
            for(int i=0; i<children.Count; i++)
                res.Add(new ExprWithString(children[i]));
            return res;
        }

        public Expr GetSimplified(int maxIterations = 100)
        {
            var a = this;

            string before = a.ToString();
            int counter = 0;
            for (int i = 0; i < maxIterations; ++i)
            {
                a = new Expr(a, SimplifyInternal);
                string after = a.ToString();
                if (after == before)
                    break;

                before = after;
                ++counter;
            }
            return a;
        }

        private void Sort(List<ExprWithString> children, List<Expr> sortAlong = null)
        {
            children.Sort(delegate (ExprWithString a, ExprWithString b)
            {
                //Special case for constants - they should appear on the very left
                if (a.E.Type == ExprType.Constant && b.E.Type != ExprType.Constant)
                    return -1;
                if (a.E.Type != ExprType.Constant && b.E.Type == ExprType.Constant)
                    return 1;

                return a.S.CompareTo(b.S);
            });

            if(sortAlong != null)
            {
                sortAlong.Clear();
                for (int i = 0; i < children.Count; ++i)
                    sortAlong.Add(children[i].E);
            }
        }

        private void CollectOfType(ExprType type, List<Expr> list)
        {
            if(this.Type != type)
            {
                list.Add(this);
                return;
            }    
            _a.CollectOfType(type, list);
            _b.CollectOfType(type, list);
        }
        private void CollectOfType(List<Expr> list, params ExprType[] allowedTypes)
        {
            if (allowedTypes.Contains(this.Type))
            {
                list.Add(this);
                return;
            }
            _a.CollectOfType(list, allowedTypes);
            _b.CollectOfType(list, allowedTypes);
        }

        //Copy constructor with optional transformation
        public Expr(Expr source, Action<Expr> transform = null)
        {
            _type = source._type;
            if (source._a != null)
                _a = new Expr(source._a, transform);
            else
                _a = null;
            if (source._b != null)
                _b = new Expr(source._b, transform);
            else
                _b = null;
            _value = source._value;

            if (transform != null)
                transform(this);
        }


        public void SetValue(double value)
        {
            if (_type == ExprType.Parameter)
                _value.Value = value;
            else
                throw new Exception("Only values expressions with type 'Parameter' can be assigned directly.");
        }

        public static Expr Constant(double value) { return new Expr() { _type = ExprType.Constant, _value = new Param(value) }; } //TODO: Mark the Param as constant?
        public static Expr Constant(double value, string name) { return new Expr() { _type = ExprType.Constant, _value = new NamedParam(value, name) }; }

        public static Expr Parameter(double value = 0) { return new Expr() { _type = ExprType.Parameter, _value = new Param(value) }; }
        public static Expr Parameter(double value, string name) { return new Expr() { _type = ExprType.Parameter, _value = new NamedParam(value, name) }; }
        public static Expr Parameter(Param p) { return new Expr() { _type = ExprType.Parameter, _value = p }; }
        public static Expr Parameter(string name) { return new Expr() { _type = ExprType.Parameter, _value = new NamedParam(0.0, name) }; }

        //public static Expr Variable(double value = 0) { return new Expr() { _type = ExprType.Parameter, _value = new VariableParam(value) }; }
        //public static Expr Variable(double value, string name) { return new Expr() { _type = ExprType.Parameter, _value = new VariableNamedParam(value, name) }; }
        //public static Expr Variable(string name) { return new Expr() { _type = ExprType.Parameter, _value = new VariableNamedParam(0.0, name) }; }

        //public static Expr Val(bool isVariable, double value = 0)
        //{
        //    return new Expr()
        //    {
        //        _type = ExprType.Parameter,
        //        _value = isVariable ? new VariableParam(value) : new Param(value)
        //    };
        //}

        //public static Expr Val(bool isVariable, double value, string name)
        //{
        //    return new Expr()
        //    {
        //        _type = ExprType.Parameter,
        //        _value = isVariable ? new VariableNamedParam(value, name) : new NamedParam(value, name)
        //    };
        //}


        //public static Expr Val(bool isVariable, string name)
        //{
        //    return new Expr()
        //    {
        //        _type = ExprType.Parameter,
        //        _value = isVariable ? new VariableNamedParam(0.0, name) : new NamedParam(0.0, name)
        //    };
        //}

        public static Expr Function(string name, params Param[] dependencies) { return new Expr() { _type = ExprType.Function, _value = new FunctionParam(name, dependencies) }; }


        public static void Scale(Expr e, double scaling)
        {
            if (e.Type == ExprType.Parameter || e.Type == ExprType.Constant)
            {
                e._value.Value *= scaling;
            }
            else if (e.Type == ExprType.Multiply && (e._a.Type == ExprType.Constant || e._a.Type == ExprType.Parameter))
            {
                e._a._value.Value *= scaling;
            }
            else
            {
                Expr copy = new Expr(e);
                e._type = ExprType.Multiply;
                e._value = null;
                e._a = Constant(scaling);
                e._b = copy;
            }
        }

        public static explicit operator Expr(Param p) { return Parameter(p); }

        public static explicit operator Expr(double value) { return Constant(value); }

        public static Expr operator +(Param a, Expr b)
        {
            return new Expr(  ExprType.Add,  Expr.Parameter(a),  b );
        }

        public static Expr operator +(Expr a, Expr b)
        {
            return new Expr( ExprType.Add, a, b );
        }
        public static Expr operator +(double a, Expr b)
        {
            return new Expr(ExprType.Add, Expr.Constant(a), b);
        }

        public static Expr operator -(Expr a)
        {
            if(a.Type == ExprType.Constant)
            {
                return Expr.Constant(-a.Value.Value);
            }

            return new Expr() { _type = ExprType.Negate, _a = a };
        }

        public static Expr operator -(double a, Expr b)
        {
            return new Expr() { _type = ExprType.Subtract, _a = Expr.Constant(a), _b = b };
        }

        public static Expr operator -(Expr a, double b)
        {
            return new Expr() { _type = ExprType.Subtract, _a = a, _b = Expr.Constant(b) };
        }

        public static Expr operator -(Expr a, Expr b)
        {
            return new Expr() { _type = ExprType.Subtract, _a = a, _b = b };
        }

        public static Expr operator *(double a, Expr b)
        {
            return new Expr(ExprType.Multiply, Expr.Constant(a), b );
        }

        public static Expr operator *(Expr a, double b)
        {
            return new Expr( ExprType.Multiply,  a,  Expr.Constant(b) );
        }

        public static Expr operator *(Expr a, Expr b)
        {
            return new Expr( ExprType.Multiply,  a,  b );
        }

        public static Expr operator /(Expr a, Expr b)
        {
            return new Expr() { _type = ExprType.Divide, _a = a, _b = b };
        }

        public static Expr Sqrt(Expr a)
        {
            return new Expr() { _type = ExprType.Sqrt, _a = a };
        }

        public static Expr Pow2(Expr a)
        {
            return new Expr() { _type = ExprType.Pow2, _a = a };
        }

        public static Expr Abs(Expr a)
        {
            return new Expr() { _type = ExprType.Abs, _a = a };
        }

        public static Expr Sign(Expr a)
        {
            return new Expr() { _type = ExprType.Sign, _a = a };
        }

        public static Expr Sin(Expr a)
        {
            return new Expr() { _type = ExprType.Sin, _a = a };
        }

        public static Expr Cos(Expr a)
        {
            return new Expr() { _type = ExprType.Cos, _a = a };
        }

        public static Expr Asin(Expr a)
        {
            return new Expr() { _type = ExprType.ASin, _a = a };
        }

        public static Expr Acos(Expr a)
        {
            return new Expr() { _type = ExprType.ACos, _a = a };
        }

        public Expr PartialDerivative(Param p)
        {
            Expr derivative = BuildPartialDerivative(p);
            return derivative == null ? Expr.Constant(0) : derivative.FoldConstants();
        }

        /// <summary>
        /// Deep copy so FoldConstants on the derivative DAG cannot alias into the primal.
        /// </summary>
        private static Expr EmbedPrimal(Expr primal)
        {
            return new Expr(primal);
        }

        private Expr BuildPartialDerivative(Param p)
        {
            switch (_type)
            {
                case ExprType.Parameter:
                    return p == _value ? Expr.Constant(1) : null;
                case ExprType.Constant:
                    return null;
                case ExprType.Function:
                    return new Expr(ExprType.Differentiate, this, Expr.Parameter(p));
                case ExprType.Differentiate:
                    return new Expr(ExprType.Differentiate, this, Expr.Parameter(p));
                case ExprType.Add:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        Expr b = _b.BuildPartialDerivative(p);
                        if (a == null) return b;
                        else if (b == null) return a;
                        else return a + b;
                    }
                case ExprType.Subtract:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        Expr b = _b.BuildPartialDerivative(p);
                        if (a == null) return b == null ? null : -b;
                        else if (b == null) return a;
                        else return a - b;
                    }
                case ExprType.Multiply:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        Expr b = _b.BuildPartialDerivative(p);
                        if (a == null) return b == null ? null : EmbedPrimal(_a) * b;
                        else if (b == null) return a * EmbedPrimal(_b);
                        else
                            return EmbedPrimal(_a) * b + a * EmbedPrimal(_b);
                    }
                case ExprType.Divide:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        Expr b = _b.BuildPartialDerivative(p);
                        Expr pb = EmbedPrimal(_b);
                        if (a == null) return b == null ? null : -(EmbedPrimal(_a) * b) / (pb * pb);
                        else if (b == null) return a / pb;
                        else return (a * pb - EmbedPrimal(_a) * b) / (pb * pb);
                    }
                case ExprType.Sqrt:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        if (a == null) return null;
                        else return a / (Expr.Constant(2) * Expr.Sqrt(EmbedPrimal(_a)));
                    }
                case ExprType.Pow2:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        if (a == null) return null;
                        else return Expr.Constant(2) * EmbedPrimal(_a) * a;
                    }
                case ExprType.Abs:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        if (a == null) return null;
                        else return Expr.Sign(EmbedPrimal(_a)) * a;
                    }
                case ExprType.Sign:
                    {
                        _a.BuildPartialDerivative(p);
                        return null;
                    }
                case ExprType.Negate:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        if (a == null) return null;
                        else return -a;
                    }
                case ExprType.Sin:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        if (a == null) return null;
                        else return Expr.Cos(EmbedPrimal(_a)) * a;
                    }
                case ExprType.Cos:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        if (a == null) return null;
                        else return -Expr.Sin(EmbedPrimal(_a)) * a;
                    }
                case ExprType.ASin:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        if (a == null) return null;
                        Expr pa = EmbedPrimal(_a);
                        return Expr.Constant(1.0) / Expr.Sqrt(Expr.Constant(1.0) - pa * pa) * a;
                    }
                case ExprType.ACos:
                    {
                        Expr a = _a.BuildPartialDerivative(p);
                        if (a == null) return null;
                        Expr pa = EmbedPrimal(_a);
                        return Expr.Constant(-1.0) / Expr.Sqrt(Expr.Constant(1.0) - pa * pa) * a;
                    }
            }
            throw new Exception();
        }

        /// <summary>
        /// AOT-safe evaluator (postfix bytecode). Same numeric contract as <see cref="Evaluate"/>.
        /// </summary>
        public Func<double> Compile()
        {
            return ExprProgram.From(this).AsFunc();
        }

        public double EvaluatePartialDerivative(Param p)
        {
            switch (_type)
            {
                case ExprType.Parameter:
                    return p == _value ? 1 : 0;
                case ExprType.Constant:
                    return 0;
                case ExprType.Function:
                    return double.NaN;
                case ExprType.Differentiate:
                    return double.NaN;
                case ExprType.Add:
                    return _a.EvaluatePartialDerivative(p) + _b.EvaluatePartialDerivative(p);
                case ExprType.Subtract:
                    return _a.EvaluatePartialDerivative(p) - _b.EvaluatePartialDerivative(p);
                case ExprType.Multiply:
                    return _a.Evaluate() * _b.EvaluatePartialDerivative(p) + _a.EvaluatePartialDerivative(p) * _b.Evaluate();
                case ExprType.Divide:
                    {
                        double b = _b.Evaluate();
                        return (_a.EvaluatePartialDerivative(p) * b - _a.Evaluate() * _b.EvaluatePartialDerivative(p)) / (b * b);
                    }
                case ExprType.Sqrt:
                    return _a.EvaluatePartialDerivative(p) / (2 * Math.Sqrt(_a.Evaluate()));
                case ExprType.Pow2:
                    return 2 * _a.Evaluate() * _a.EvaluatePartialDerivative(p);
                case ExprType.Abs:
                    {
                        double a = _a.Evaluate();
                        if (a == 0)
                        {
                            //Dangerous, derivative is not really defined here...
                        }
                        return Math.Sign(a) * _a.EvaluatePartialDerivative(p);
                    }
                case ExprType.Sign:
                    {
                        double a = _a.Evaluate();
                        if (a == 0)
                        {
                            //Dangerous, derivative is infinity here...
                        }
                        return 0;
                    }
                case ExprType.Negate:
                    return -_a.EvaluatePartialDerivative(p);
                case ExprType.Sin:
                    return Math.Cos(_a.Evaluate()) * _a.EvaluatePartialDerivative(p);
                case ExprType.Cos:
                    return -Math.Sin(_a.Evaluate()) * _a.EvaluatePartialDerivative(p);
                case ExprType.ASin:
                    {
                        double a = _a.Evaluate();
                        return 1.0 / Math.Sqrt(1.0 - a * a) * _a.EvaluatePartialDerivative(p);
                    }
                case ExprType.ACos:
                    {
                        double a = _a.Evaluate();
                        return -1.0 / Math.Sqrt(1.0 - a * a) * _a.EvaluatePartialDerivative(p);
                    }
            }
            throw new Exception();
        }

        //TODO: Could also provide Derivative as expression: this would allow to derive as many times as the caller needs
        //TODO: Compile into lambda expression

        //public double Evaluate() { return Evaluate(); }
        public double Evaluate()
        {
            switch (_type)
            {
                case ExprType.Parameter:
                    return _value.Value;
                case ExprType.Constant:
                    return _value.Value;
                case ExprType.Function:
                    return double.NaN;
                case ExprType.Differentiate:
                    return double.NaN;
                case ExprType.Add:
                    return _a.Evaluate() + _b.Evaluate();
                case ExprType.Subtract:
                    return _a.Evaluate() - _b.Evaluate();
                case ExprType.Multiply:
                    return _a.Evaluate() * _b.Evaluate();
                case ExprType.Divide:
                    return _a.Evaluate() / _b.Evaluate();
                case ExprType.Sqrt:
                    {
                        double a = _a.Evaluate();
                        if (a < 0)
                            throw new Exception();
                        return Math.Sqrt(a);
                    }
                case ExprType.Pow2:
                    {
                        double d = _a.Evaluate();
                        return d * d;
                    }
                case ExprType.Abs:
                    return Math.Abs(_a.Evaluate());
                case ExprType.Sign:
                    return Math.Sign(_a.Evaluate());
                case ExprType.Negate:
                    return -_a.Evaluate();
                case ExprType.Sin:
                    return Math.Sin(_a.Evaluate());
                case ExprType.Cos:
                    return Math.Cos(_a.Evaluate());
                case ExprType.ASin:
                    {
                        double a = _a.Evaluate();
                        if (a > 1 || a < -1)
                            throw new Exception();
                        return Math.Asin(a);
                    }
                case ExprType.ACos:
                    {
                        double a = _a.Evaluate();
                        if (a > 1 || a < -1)
                            throw new Exception();
                        return Math.Acos(a);
                    }
            }
            return 0;
        }

        ////Sorts sums and products
        //public void Sort()
        //{

        //}


        public override string ToString()
        {
            return ToString(ExprType.None, null, 0);
        }

        private string ApplyBracket(string s, ExprType parentType)
        {
            //return "(" + s + ")";


            if (parentType == ExprType.None)
                return s;
            //

            if ((Type == ExprType.Add || Type == ExprType.Subtract) &&
                (parentType == ExprType.Add))
                return s;

            if ((Type == ExprType.Multiply || Type == ExprType.Divide) &&
                (parentType == ExprType.Multiply || parentType == ExprType.Divide))
                return s;

            if ((Type == ExprType.Multiply || Type == ExprType.Divide) &&
               (parentType == ExprType.Add))
                return s; //Point before line rule applies in this case

            return "(" + s + ")";
        }

        public class SubexpressionCache
        {
            public class CacheEntry
            {
                public string Value;
                public Expr expression;
                public int UsageCounter;              

                public CacheEntry(string evaluate, Expr expression, int usageCounter)
                {
                    Value = evaluate;
                    UsageCounter = usageCounter;
                    this.expression = expression;
                }

                public override string ToString()
                {
                    return Value;
                }
            }

            List<CacheEntry> unique = new List<CacheEntry>();
            bool collectRun = true;

            private static int IndexOf(List<CacheEntry> list, string s)
            {
                for(int i= list.Count-1; i>=0; i--)
                    if (list[i].Value == s) 
                        return i;
                return -1;
            }

            public string Register(string text, Expr e, int depth)
            {
                string n;
                if (collectRun)
                {
                    int id = IndexOf(unique, text);
                    if (id < 0)
                    {
                        id = unique.Count;
                        unique.Add(new CacheEntry(text, e, 1));
                    }
                    else
                        unique[id].UsageCounter++;
                    n = Name(id); 
                    if (n == "@v")
                    {

                    }
                }
                else
                {
                    if (depth == 0)
                        return text;

                    int id = IndexOf(unique, text);
                    if (id < 0)
                        return text;

                    var entry = unique[id];
                    if (entry.UsageCounter == 1)
                        return text;

                    n = Name(id); 
                    if (n == "@v")
                    {

                    }
                }

                

                return n;
            }

            private static string Name(int id)
            {
                string abc = "abcdefghijklmnopqrstuvwxyz";
                int i = id;
                string name = "";
                while(true)
                {
                    name = abc[i%abc.Length] + name;
                    i /= abc.Length;
                    if (i == 0)
                        break;
                }
                return "_"+name;
            }

            public string Collect(Expr e)
            {
                collectRun = true;
                e.ToString(ExprType.None, this, 1);
                return Name(unique.Count - 1);
            }
            public List<string> Collect(IEnumerable<Expr> e)
            {
                List<string> result = new List<string>();
                foreach (var ee in e)
                {
                    result.Add(Collect(ee));
                }               
                return result;
            }
            public static int CountSubstring(string text, string value)
            {
                return Regex.Matches(text, @"\b(" + value + @")\b").Count;
            }
            public static int CountChar(string text, char c)
            {
                int counter = 0;
                for (int i = 0; i < text.Length; i++)
                    if (text[i] == c)
                        ++counter;
                return counter;
            }

            private struct NameAndIndex
            {
                public string Name;
                public int Index;

                public NameAndIndex(string name, int index)
                {
                    Name = name;
                    Index = index;
                }
            }

            public string GenerateCode(List<string> outputNames)
            {
                bool directExport = true;

                //if(directExport)
                //{


                //    return;
                //}


                //cache.collectRun = true;
                //e.ToString(e.Type, cache, 1);

                collectRun = false;
                string code;
                List<NameAndIndex> names = new List<NameAndIndex>();

                {
                    StringBuilder sb = new StringBuilder();
                    for (int i = 0; i < unique.Count; i++)
                    {
                        var c = unique[i];
                        if (c.UsageCounter == 1 && !outputNames.Contains(Name(i)))
                            continue;

                        sb.Append("var ");
                        string name = Name(i);
                        names.Add(new NameAndIndex(name, i));
                        sb.Append(name);
                        sb.Append(" = ");
                        sb.Append(c.expression.ToString(ExprType.None, this, 0));
                        sb.AppendLine(";");
                    }
                    code = sb.ToString();
                }


                List<string> toReplace = new List<string>();
                for(int i=0;i<names.Count;++i)
                {
                    var n = names[i];
                    int count = CountSubstring(code, n.Name);

                    //if (count < 2)
                    //    throw new Exception();
                    if (count == 2 && !outputNames.Contains(n.Name))
                    {
                        //unique[n.Index].UsageCounter = 1;


                        toReplace.Add(n.Name);
                    }

                    //Save registers for cheap operations
                    if(count == 3 && !outputNames.Contains(n.Name))
                    {
                        var r = new Regex(@"var " + n.Name + @" = (?<value>[^;]+);(\n|\r\n?)").Match(code);
                        string value = r.Groups["value"].Value;
                                                
                        int c = CountChar(value, '+') + CountChar(value, '-') + CountChar(value, '*') /*+ CountChar(value, '/')*/;
                        if(c==1)
                            toReplace.Add(n.Name);
                    }
                }

                for(int i=0;i<toReplace.Count;++i)
                {
                    var n = toReplace[i];
                    var r = new Regex(@"var "+n+ @" = (?<value>[^;]+);(\n|\r\n?)").Match(code);
                    string value = r.Groups["value"].Value;
                    code = code.Remove(r.Index, r.Length);
                    code = Regex.Replace(code, @"\b(" + n + @")\b", "("+value+")");
                   
                }

                //{
                //    StringBuilder sb = new StringBuilder();
                //    for (int i = 0; i < unique.Count; i++)
                //    {
                //        var c = unique[i];
                //        string name = Name(i);
                //        if (name == "_dk")
                //        {

                //        }

                //        if (c.UsageCounter == 1 && !outputNames.Contains(Name(i)))
                //            continue;

                //        sb.Append("var ");
                        
                //        names.Add(new NameAndIndex(name, i));
                //        sb.Append(name);
                //        sb.Append(" = ");
                //        sb.Append(c.expression.ToString(c.expression.Type, this, 0));
                //        sb.AppendLine(";");
                //    }
                //    code = sb.ToString();
                //}

                collectRun = true;
                return code;
            }
        }

        private string ToString(ExprType parentType, SubexpressionCache cache, int depth)
        {
            const string format = "";// "0.###";
            string result = null;
            switch (_type)
            {
                case ExprType.Parameter:
                    result = _value.GetNameOrValue(format); break; // _value.Value.ToString(format);
                case ExprType.Constant:
                    result = _value.Value.ToString(format); break;
                case ExprType.Function:
                    result = _value.ToString(); break;
                case ExprType.Differentiate:
                    result = "Diff("+_a.ToString(Type, cache, depth + 1) +", "+_b._value.GetNameOrValue(format) + ")"; break;
                case ExprType.Add:
                    result = ApplyBracket( _a.ToString(Type, cache, depth + 1) + " + " + _b.ToString(Type, cache, depth + 1), parentType ); break;
                case ExprType.Subtract:
                    result = ApplyBracket(_a.ToString(Type, cache, depth + 1) + " - " + _b.ToString(Type, cache, depth + 1), parentType); break;
                case ExprType.Multiply:
                    result = ApplyBracket(_a.ToString(Type, cache, depth + 1) + " * " + _b.ToString(Type, cache, depth + 1), parentType); break;
                case ExprType.Divide:
                    result = ApplyBracket(_a.ToString(Type, cache, depth + 1) + " / " + _b.ToString(Type, cache, depth + 1), parentType); break;
                case ExprType.Sign:
                    result = "Sign(" + _a.ToString(Type, cache, depth + 1) + ")"; break;
                case ExprType.Abs:
                    result = "Abs(" + _a.ToString(Type, cache, depth + 1) + ")"; break;
                case ExprType.Sqrt:
                    result = "Sqrt(" + _a.ToString(Type, cache, depth + 1) + ")"; break;
                case ExprType.Pow2:
                    result = "Pow2(" + _a.ToString(Type, cache, depth + 1) + ")"; break;
                case ExprType.Negate:
                    result = "-(" + _a.ToString(Type, cache, depth + 1) + ")"; break;
                case ExprType.Sin:
                    result = "Sin(" + _a.ToString(Type, cache, depth + 1) + ")"; break;
                case ExprType.Cos:
                    result = "Cos(" + _a.ToString(Type, cache, depth + 1) + ")"; break;
                case ExprType.ASin:
                    result = "ASin(" + _a.ToString(Type, cache, depth + 1) + ")"; break;
                case ExprType.ACos:
                    result = "ACos(" + _a.ToString(Type, cache, depth + 1) + ")"; break;
            }
            if (result == null)
                throw new Exception();

            if(cache != null && _type != ExprType.Parameter && 
                _type != ExprType.Constant && _type != ExprType.Function)
                result = cache.Register(result, this, depth);

            return result;
        }

        public Expr FoldConstants()
        {
            switch (_type)
            {
                case ExprType.Parameter:
                case ExprType.Constant:
                case ExprType.Function:
                    return this;

                case ExprType.Subtract:
                case ExprType.Add:
                    {
                        Expr a = _a.FoldConstants();
                        Expr b = _b.FoldConstants();
                        if (a._type == ExprType.Constant && b._type == ExprType.Constant)
                            return Expr.Constant(EvalBinary(_type, a._value.Value, b._value.Value));
                        if (a._type == ExprType.Constant && a._value.Value == 0.0)
                            return _type == ExprType.Subtract ? -b : b;
                        if (b._type == ExprType.Constant && b._value.Value == 0.0)
                            return a;
                        if (a == _a && b == _b)
                            return this;
                        return new Expr(_type, a, b);
                    }
                case ExprType.Multiply:
                    {
                        Expr a = _a.FoldConstants();
                        Expr b = _b.FoldConstants();
                        if (a._type == ExprType.Constant && b._type == ExprType.Constant)
                            return Expr.Constant(EvalBinary(_type, a._value.Value, b._value.Value));
                        if (a._type == ExprType.Constant && a._value.Value == 1.0)
                            return b;
                        if (b._type == ExprType.Constant && b._value.Value == 1.0)
                            return a;
                        if (a._type == ExprType.Constant && a._value.Value == -1.0)
                            return -b;
                        if (b._type == ExprType.Constant && b._value.Value == -1.0)
                            return -a;
                        if (a._type == ExprType.Constant && a._value.Value == 0.0)
                            return Expr.Constant(0.0);
                        if (b._type == ExprType.Constant && b._value.Value == 0.0)
                            return Expr.Constant(0.0);
                        if (a == _a && b == _b)
                            return this;
                        return new Expr(_type, a, b);
                    }
                case ExprType.Divide:
                    {
                        Expr a = _a.FoldConstants();
                        Expr b = _b.FoldConstants();
                        if (a._type == ExprType.Constant && b._type == ExprType.Constant)
                            return Expr.Constant(EvalBinary(_type, a._value.Value, b._value.Value));
                        // 1/x must stay a quotient; multiplying's 1*x → x rule does not apply here.
                        if (b._type == ExprType.Constant && b._value.Value == 1.0)
                            return a;
                        if (b._type == ExprType.Constant && b._value.Value == -1.0)
                            return -a;
                        if (a._type == ExprType.Constant && a._value.Value == 0.0)
                            return Expr.Constant(0.0);
                        if (b._type == ExprType.Constant && b._value.Value == 0.0)
                            throw new Exception("Division by zero");
                        if (a == _a && b == _b)
                            return this;
                        return new Expr(_type, a, b);
                    }
                case ExprType.Sqrt:
                case ExprType.Pow2:
                case ExprType.Negate:
                case ExprType.Sin:
                case ExprType.Cos:
                case ExprType.ASin:
                case ExprType.ACos:
                case ExprType.Abs:
                    {
                        Expr a = _a.FoldConstants();
                        if (a._type == ExprType.Constant)
                            return Expr.Constant(EvalUnary(_type, a._value.Value));
                        if (a == _a)
                            return this;
                        return new Expr(_type, a, null);
                    }
            }
            return this;
        }

        private static double EvalBinary(ExprType type, double a, double b)
        {
            switch (type)
            {
                case ExprType.Add: return a + b;
                case ExprType.Subtract: return a - b;
                case ExprType.Multiply: return a * b;
                case ExprType.Divide: return a / b;
                default: throw new Exception();
            }
        }

        private static double EvalUnary(ExprType type, double a)
        {
            switch (type)
            {
                case ExprType.Sqrt: return Math.Sqrt(a);
                case ExprType.Pow2: return a * a;
                case ExprType.Negate: return -a;
                case ExprType.Sin: return Math.Sin(a);
                case ExprType.Cos: return Math.Cos(a);
                case ExprType.ASin: return Math.Asin(a);
                case ExprType.ACos: return Math.Acos(a);
                case ExprType.Abs: return Math.Abs(a);
                default: throw new Exception();
            }
        }

        public int NumSubElements()
        {
            switch (_type)
            {
                case ExprType.Parameter:
                case ExprType.Constant:
                case ExprType.Function:
                    return 0;

                case ExprType.Differentiate:
                    return 2;

                case ExprType.Subtract:
                case ExprType.Add:
                case ExprType.Multiply:
                case ExprType.Divide:
                    return 2;
                case ExprType.Sqrt:
                case ExprType.Pow2:
                case ExprType.Negate:
                case ExprType.Sin:
                case ExprType.Cos:
                case ExprType.ASin:
                case ExprType.ACos:
                case ExprType.Abs:
                    return 1;
            }
            throw new Exception();
            return -1;
        }

        public HashSet<Param> GetContainedParams()
        {
            HashSet<Param> set = new HashSet<Param>();
            GetContainedParams(set);
            return set;
        }

        private void GetContainedParams(HashSet<Param> set)
        {
            if (_type == ExprType.Parameter)
            {
                // Skip frozen parameters - they are treated as constants
                if (!_value.Frozen)
                    set.Add(_value);
                return;
            }
            else if (_type == ExprType.Constant)
                return;
            else if(_type == ExprType.Function)
            {
                foreach (var v in (_value as FunctionParam).Dependencies)
                {
                    // Skip frozen parameters
                    if (!v.Frozen)
                        set.Add(v);
                }
            }    
            else
            {
                int num = NumSubElements();
                switch (num)
                {
                    case 1:
                        _a.GetContainedParams(set);
                        break;
                    case 2:
                        _a.GetContainedParams(set);
                        _b.GetContainedParams(set);
                        break;
                }
            }
        }

        public void MakeParamConstant(Param p)
        {
            AssignToThis(FreezeParamAsConstant(p));
        }

        /// <summary>
        /// Non-mutating: Parameter nodes matching <paramref name="p"/> become constants
        /// holding the current value.
        /// </summary>
        public Expr FreezeParamAsConstant(Param p)
        {
            if (_type == ExprType.Parameter)
                return _value == p ? Expr.Constant(p.Value) : this;
            if (_type == ExprType.Constant || _type == ExprType.Function)
                return this;

            int num = NumSubElements();
            if (num == 1)
            {
                Expr a = _a.FreezeParamAsConstant(p);
                if (a == _a)
                    return this;
                return new Expr(_type, a, null);
            }
            if (num == 2)
            {
                Expr a = _a.FreezeParamAsConstant(p);
                Expr b = _b.FreezeParamAsConstant(p);
                if (a == _a && b == _b)
                    return this;
                return new Expr(_type, a, b);
            }
            return this;
        }

        /// <summary>
        /// Non-mutating replacement of a parameter with an expression (used by algebraic reductions).
        /// </summary>
        public Expr ReplaceParam(Param oldParam, Expr replacement)
        {
            if (_type == ExprType.Parameter)
                return _value == oldParam ? replacement : this;
            if (_type == ExprType.Constant)
                return this;
            if (_type == ExprType.Function)
            {
                (_value as FunctionParam).Substitute(oldParam, replacement.Type == ExprType.Parameter ? replacement.Value : oldParam);
                return this;
            }

            int num = NumSubElements();
            if (num == 1)
            {
                Expr a = _a.ReplaceParam(oldParam, replacement);
                if (a == _a)
                    return this;
                return new Expr(_type, a, null);
            }
            if (num == 2)
            {
                Expr a = _a.ReplaceParam(oldParam, replacement);
                Expr b = _b.ReplaceParam(oldParam, replacement);
                if (a == _a && b == _b)
                    return this;
                return new Expr(_type, a, b);
            }
            return this;
        }

        public bool DependsOn(Param p)
        {
            if (_type == ExprType.Parameter)
                return _value == p;
            else if (_type == ExprType.Constant)
                return false;
            else if(_type == ExprType.Function)
            {
                if (_value == p)
                    return true;
                return (p as FunctionParam).DependsOn(p);
            }

            int num = NumSubElements();
            switch (num)
            {
                case 1:
                    return _a.DependsOn(p);
                case 2:
                    return _a.DependsOn(p) || _b.DependsOn(p);
            }
            return false;
        }

        //public Expr BuildPartialDerivative(Param p) { return BuildPartialDerivative(p); }

        //public Expr DeepCopyWithParamsAsPointers() { return new Expr(this); }

        public Expr FindFactorOf(Param p)
        {
            int indexer = 0;
            Expr result = Expr.Constant(0);
            while(true)
            {
                var sum = new Expr(this).FindFactorOfInPlace(p, indexer);
                if (sum==null)                
                    break;
                result += sum;
                ++indexer;
            }
            result.FoldConstants();
            return result;
        }
        private Expr FindFactorOfInPlace(Param p, int index)
        {
            List<Expr> stack = new List<Expr>();
            List<List<Expr>> all = new List<List<Expr>>();
            FindAll(p, stack, all);
            if (index >= all.Count)
                return null;
            if (stack.Count != 0)
                throw new Exception();
            
            stack = all[index];

            var root = stack.Pop(); //Remove p from top of stack
            

            //List<Expr> factors = new List<Expr>();
            var prev = root;
            List<Expr> setZero = new List<Expr>();
            while (stack.Count > 0)
            {
                var e = stack.Pop();
                int num = e.NumSubElements();
                if (e.Type == ExprType.Multiply || e.Type == ExprType.Divide  || e.Type == ExprType.Negate)
                {
                    //factors.Add(e);
                }
                else if(num == 2) //if (e.Type == ExprType.Add || e.Type == ExprType.Subtract)
                {
                    int count = 0;
                    if (e.a != prev/*!e.a.Identical( prev)*/)
                    {
                        setZero.Add(e.a);
                        ++count;
                    }
                    if (e.b != prev/*!e.b.Identical( prev)*/)
                    {
                        setZero.Add(e.b);
                        ++count;
                    }
                    if (count != 1)
                        throw new Exception();
                    /*if (!e.a.DependsOn(p))
                        e.a.SetToConst(0);
                    if(!e.b.DependsOn(p))
                        e.b.SetToConst(0);*/
                }
                prev = e;
            }

            for (int i = 0; i < setZero.Count; ++i)
                setZero[i].SetToConst(0);
            root.SetToConst(1.0);

            Expr result = prev;// factors[factors.Count - 1]; 
            //for (int i = 0; i < factors.Count; ++i)
            //    result = result * factors[i];

            result.FoldConstants();

            return result;
        }

        public void SetToConst(double value)
        {
            _type = ExprType.Constant;
            _value = new Param(value);
            _a = null;
            _b = null;
        }

        //public Expr FindFactorOf(Param p)
        //{
        //    Stack<Expr> stack = new Stack<Expr>();
        //    bool found = Find(p, stack);

        //    if (found)
        //        stack.Pop(); //Remove p from top of stack

        //    Expr lastValid = Expr.Constant(1.0);
        //    while(stack.Count>0)
        //    {
        //        var e = stack.Pop();
        //        if(e.Type != ExprType.Multiply && e.Type != ExprType.Divide)
        //        {
        //            break;
        //        }
        //        lastValid = e;
        //    }
        //    return lastValid;
        //}

        public Expr DeepCopyExceptParams()
        {
            return new Expr(this);
        }

        //Multiplies factors into sums
        //public Expr GetExpanded()
        //{

        //}

        private bool Find(Param p, List<Expr> stack)
        {
            stack.Add(this);
            if (_value == p)
                return true;

            int num = NumSubElements();
            switch (num)
            {
                case 1:
                    {
                        if (_a.Find(p, stack))
                            return true;
                    }
                    break;
                case 2:
                    {
                        if (_a.Find(p, stack))
                            return true;
                        if (_b.Find(p, stack))
                            return true;
                    }
                    break;
            }
            stack.RemoveAt(stack.Count - 1);
            return false;
        }
        private void FindAll(Param p, List<Expr> stack, List<List<Expr>> result)
        {
            stack.Add(this);
            if (_value == p)
            {
                result.Add(new List<Expr>(stack) /*Copy(stack)*/);
                stack.Pop();
                return;
            }

            int num = NumSubElements();
            switch (num)
            {
                case 1:
                    {
                        _a.FindAll(p, stack, result);
                    }
                    break;
                case 2:
                    {
                        _a.FindAll(p, stack, result);
                        _b.FindAll(p, stack, result);
                    }
                    break;
            }
            stack.Pop();
        }

        private List<Expr> Copy(List<Expr> stack)
        {
            List<Expr> res = new List<Expr>(stack.Count);
            for (int i = 0;i<stack.Count;++i)
                res.Add(new Expr(stack[i]));
            return res;
        }

        private Expr Find(Param p)
        {
            if (_type == ExprType.Parameter && _value == p)
                return this;

            int num = NumSubElements();
            switch (num)
            {
                case 1:
                    return _a.Find(p);
                case 2:
                    {
                        var f = _a.Find(p);
                        if (f != null)
                            return f;
                        return _b.Find(p);
                    }
            }
            return null;
        }

        public void Substitute(Expr oldExpr, Expr newExpr)
        {           
            if (Equal(oldExpr, this))
            {
                this._a = newExpr._a;
                this._b = newExpr._b;
                this._value = newExpr._value;
                this._type = newExpr._type;
                return;
            }
            
            int num = NumSubElements();
            switch (num)
            {
                case 1:
                    _a.Substitute(oldExpr, newExpr);
                    break;
                case 2:
                    _a.Substitute(oldExpr, newExpr);
                    _b.Substitute(oldExpr, newExpr);
                    break;
            }
        }



        public bool Identical(Expr other)
        {
            if (other == null)
                return false;

            if (_type != other._type) return false;
            if (_value != other._value) return false;
            if ((_a==null) != (other._a == null))            
                return false;
            if ((_b == null) != (other._b == null))
                return false;

            if (_a != null && !_a.Identical(other._a)) return false;
            if (_b != null && !_b.Identical(other._b)) return false;
            return true;
        }
        private bool Equal(Expr a, Expr b)
        {
            return a.Type == b.Type && a.Value == b.Value && a.ToString() == b.ToString();
        }

        public void Substitute(Param oldParam, Param newParam)
        {
            if (_type == ExprType.Parameter && _value == oldParam)
                _value = newParam;
            if(_type == ExprType.Function)
                (_value as FunctionParam).Substitute(oldParam, newParam);

            int num = NumSubElements();
            switch (num)
            {
                case 1:
                    _a.Substitute(oldParam, newParam);
                    break;
                case 2:
                    _a.Substitute(oldParam, newParam);
                    _b.Substitute(oldParam, newParam);
                    break;
            }
        }

        //-----------------------------------------------------------------------------
        // If the expression references only one parameter that appears in pl, then
        // return that parameter. If no param is referenced, then return NO_PARAMS.
        // If multiple params are referenced, then return MULTIPLE_PARAMS.
        //-----------------------------------------------------------------------------
        public void ReferencedParams(List<Param> pl)
        {
            if (_type == ExprType.Parameter)
            {
                pl.Add(_value);
                return;
                //if(_/*pl.FindByIdNoOops(x.parh)*/) {
                //    return x.parh;
                //} else {
                //    return;// NO_PARAMS;
                //}
            }
            if(_type == ExprType.Function)
            {
                pl.AddRange((_value as FunctionParam).Dependencies);
                return;
            }
            //if(op == PARAM_PTR) oops();

            int c = NumSubElements();
            if (c == 0)
            {
                return;
            }
            else if (c == 1)
            {
                a.ReferencedParams(pl);
                return;
            }
            else if (c == 2)
            {
                //Param pa, pb;
                /*pa =*/
                a.ReferencedParams(pl);
                /*pb =*/
                b.ReferencedParams(pl);
                return;
                //if (pa.v == NO_PARAMS.v)
                //{
                //    return pb;
                //}
                //else if (pb.v == NO_PARAMS.v)
                //{
                //    return pa;
                //}
                //else if (pa.v == pb.v)
                //{
                //    return pa; // either, doesn't matter
                //}
                //else
                //{
                //    return MULTIPLE_PARAMS;
                //}
            }
            else throw new Exception();
        }

        public IEnumerator<Param> GetEnumerator()
        {
            if (_type == ExprType.Parameter)
                yield return _value;
            if (_type == ExprType.Function)
            {
                FunctionParam fp = _value as FunctionParam;
                foreach (var v in fp.Dependencies)
                    yield return v;
            }

            int numArgs = NumSubElements();
            if (numArgs > 0)
                foreach (Param p in _a)
                    yield return p;
            if (numArgs > 1)
                foreach (Param p in _b)
                    yield return p;
        }
        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() { return GetEnumerator(); }

        public Param AsParam()
        {
            if (Type != ExprType.Parameter && Type != ExprType.Constant)
                throw new Exception();

            return _value;
        }
    }
}
