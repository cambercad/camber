namespace GeoCore
{
    public struct ValuePair<T, U>
    {
        public T Item1;
        public U Item2;

        public ValuePair(T item1, U item2)
        {
            Item1 = item1;
            Item2 = item2;
        }

        public override string ToString()
        {
            return Item1.ToString() + " ; " + Item2.ToString();
        }
    }
}
