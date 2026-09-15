using GeoCore;

namespace Geo
{
    public class ObjMaterial
    {
        public string Name;
        public float Ns;
        public float Ni;
        public float d;
        public float Tr;
        public Vec3D Tf;
        public int illum;
        public Vec3D Ka;
        public Vec3D Kd;
        public Vec3D Ks;
        public Vec3D Ke;
        public string map_Ka;
        public string map_Kd;
        public string map_d;
        public string map_bump;
        public string bump;
    }
}
