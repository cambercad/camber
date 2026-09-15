namespace NURBS
{
    public static class ArrayExtensions
    {
        public static T[] Copy<T>(this T[] array)
        {
            T[] copy = new T[array.Length];
            Array.Copy(array, copy, array.Length);
            return array;
        }

        public static int[] InvertMap(int[] map)
        {
            int l = map.Length;
            int[] inverse = new int[l];
            for (int i = 0; i < l; ++i)
                inverse[map[i]] = i;
            return inverse;
        }

        public static List<int> InvertMap(List<int> map)
        {
            int l = map.Count;
            List<int> inverse = new List<int>(l);
            for (int i = 0; i < l; ++i) 
                inverse.Add(0);
            for (int i = 0; i < l; ++i)
                inverse[map[i]] = i;
            return inverse;
        }

        public static void Sort<T, U>(IList<T> keys, IList<U> items, Func<T, T, int> comparer)
        {
            Stack<KeyValuePair<int, int>> stack = new Stack<KeyValuePair<int, int>>();
            stack.Push(new KeyValuePair<int, int>(0, keys.Count - 1));
            while (stack.Count > 0)
            {
                KeyValuePair<int, int> kvp = stack.Pop();
                int l = kvp.Key;
                int r = kvp.Value;
                if (l < r)
                {
                    int divider = Split(l, r, ref keys, ref items, comparer);
                    stack.Push(new KeyValuePair<int, int>(l, divider - 1));
                    stack.Push(new KeyValuePair<int, int>(divider + 1, r));
                    //Quicksort(l, teiler - 1, ref keys);
                    //Quicksort(teiler + 1, r, ref keys);
                }
            }
        }
        //http://de.wikibooks.org/wiki/Algorithmensammlung:_Sortierverfahren:_Quicksort
        private static int Split<T, U>(int l, int r, ref IList<T> keys, ref IList<U> items, Func<T, T, int> comparer)
        {
            int i = l;
            //Starte mit j links vom Pivotelement
            int j = r - 1;
            T pivot = keys[r];

            do
            {
                //Suche von links ein Element, welches größer als das Pivotelement ist
                while (comparer(keys[i], pivot) <= 0 && i < r)
                    ++i;

                //Suche von rechts ein Element, welches kleiner als das Pivotelement ist
                while (comparer(keys[j], pivot) >= 0 && j > l)
                    --j;

                if (i < j)
                {
                    T a = keys[i];
                    keys[i] = keys[j];
                    keys[j] = a;
                    U b = items[i];
                    items[i] = items[j];
                    items[j] = b;
                }

            } while (i < j);
            //solange i an j nicht vorbeigelaufen ist 

            // Tausche Pivotelement (daten[rechts]) mit neuer endgültiger Position (daten[i])

            if (comparer(keys[i], pivot) > 0)
            {
                T a = keys[i];
                keys[i] = keys[r];
                keys[r] = a;
                U b = items[i];
                items[i] = items[r];
                items[r] = b;
            }
            return i; // gib die Position des Pivotelements zurück
        }

        public static void Sort<T, U, V>(IList<T> keys, IList<U> items1, IList<V> items2, Func<T, T, int> comparer)
        {
            Stack<KeyValuePair<int, int>> stack = new Stack<KeyValuePair<int, int>>();
            stack.Push(new KeyValuePair<int, int>(0, keys.Count - 1));
            while (stack.Count > 0)
            {
                KeyValuePair<int, int> kvp = stack.Pop();
                int l = kvp.Key;
                int r = kvp.Value;
                if (l < r)
                {
                    int divider = Split(l, r, ref keys, ref items1, ref items2, comparer);
                    stack.Push(new KeyValuePair<int, int>(l, divider - 1));
                    stack.Push(new KeyValuePair<int, int>(divider + 1, r));
                    //Quicksort(l, teiler - 1, ref keys);
                    //Quicksort(teiler + 1, r, ref keys);
                }
            }
        }
        //http://de.wikibooks.org/wiki/Algorithmensammlung:_Sortierverfahren:_Quicksort
        private static int Split<T, U, V>(int l, int r, ref IList<T> keys, ref IList<U> items1, ref IList<V> items2, Func<T, T, int> comparer)
        {
            int i = l;
            //Starte mit j links vom Pivotelement
            int j = r - 1;
            T pivot = keys[r];

            do
            {
                //Suche von links ein Element, welches größer als das Pivotelement ist
                while (comparer(keys[i], pivot) <= 0 && i < r)
                    ++i;

                //Suche von rechts ein Element, welches kleiner als das Pivotelement ist
                while (comparer(keys[j], pivot) >= 0 && j > l)
                    --j;

                if (i < j)
                {
                    T a = keys[i];
                    keys[i] = keys[j];
                    keys[j] = a;
                    U b = items1[i];
                    items1[i] = items1[j];
                    items1[j] = b;
                    V c = items2[i];
                    items2[i] = items2[j];
                    items2[j] = c;
                }

            } while (i < j);
            //solange i an j nicht vorbeigelaufen ist 

            // Tausche Pivotelement (daten[rechts]) mit neuer endgültiger Position (daten[i])

            if (comparer(keys[i], pivot) > 0)
            {
                T a = keys[i];
                keys[i] = keys[r];
                keys[r] = a;
                U b = items1[i];
                items1[i] = items1[r];
                items1[r] = b;
                V c = items2[i];
                items2[i] = items2[r];
                items2[r] = c;
            }
            return i; // gib die Position des Pivotelements zurück
        }


        public static int[] Sort<T>(IList<T> keys, Func<T, T, int> comparer)
        {
            int[] indexer = new int[keys.Count];
            for (int i = 0; i < keys.Count; ++i)
                indexer[i] = i;

            Stack<KeyValuePair<int, int>> stack = new Stack<KeyValuePair<int, int>>();
            stack.Push(new KeyValuePair<int, int>(0, keys.Count - 1));
            while (stack.Count > 0)
            {
                KeyValuePair<int, int> kvp = stack.Pop();
                int l = kvp.Key;
                int r = kvp.Value;
                if (l < r)
                {
                    int divider = Split(l, r, ref keys, indexer, comparer);
                    stack.Push(new KeyValuePair<int, int>(l, divider - 1));
                    stack.Push(new KeyValuePair<int, int>(divider + 1, r));
                    //Quicksort(l, teiler - 1, ref keys);
                    //Quicksort(teiler + 1, r, ref keys);
                }
            }
            return indexer;
        }
        //http://de.wikibooks.org/wiki/Algorithmensammlung:_Sortierverfahren:_Quicksort
        private static int Split<T>(int l, int r, ref IList<T> keys, int[] indexer, Func<T, T, int> comparer)
        {
            int i = l;
            //Starte mit j links vom Pivotelement
            int j = r - 1;
            T pivot = keys[indexer[r]];

            do
            {
                //Suche von links ein Element, welches größer als das Pivotelement ist
                while (comparer(keys[indexer[i]], pivot) <= 0 && i < r)
                    ++i;

                //Suche von rechts ein Element, welches kleiner als das Pivotelement ist
                while (comparer(keys[indexer[j]], pivot) >= 0 && j > l)
                    --j;

                if (i < j)
                {                    
                    int b = indexer[i];
                    indexer[i] = indexer[j];
                    indexer[j] = b;
                }

            } while (i < j);
            //solange i an j nicht vorbeigelaufen ist 

            // Tausche Pivotelement (daten[rechts]) mit neuer endgültiger Position (daten[i])

            if (comparer(keys[indexer[i]], pivot) > 0)
            {                
                int b = indexer[i];
                indexer[i] = indexer[r];
                indexer[r] = b;
            }
            return i; // gib die Position des Pivotelements zurück
        }

        public static void Shuffle<T>(this IList<T> list)
        {
            Random rng = new Random();
            int n = list.Count;
            while (n > 1)
            {
                n--;
                int k = rng.Next(n + 1);
                T value = list[k];
                list[k] = list[n];
                list[n] = value;
            }
        }

        /*//  Sorts an IList<T> in place.
        public static void Sort<T>(this IList<T> list, int index, int count)
        {
            ArrayList.Adapter((IList)list).Sort(index, count);
        }*/
    }
}
