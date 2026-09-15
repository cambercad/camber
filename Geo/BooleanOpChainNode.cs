using CSG;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Geo
{
    public class BooleanOpChainNode
    {        
        // MeshA is the result from the previous operation
        public AnchorMesh MeshB;
        public BooleanOp Operation;
    }
}
