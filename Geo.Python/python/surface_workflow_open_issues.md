# Surface workflow status

- **Closed-loop trimming on curved sheets is covered.** Both an analytic cylinder wall and a sampled circle extrusion now split at a horizontal plane crossing at `z=7.123`; see `SurfaceWorkflowTests.SurfaceSplitRecognizesAClosedIntersectionLoopOnACurvedSheet` and `SurfaceWorkflowTests.SampledCylindricalSheetSplitsAtAClosedIntersectionLoop`. The earlier no-op could not be reproduced in the current resolver, so no resolver workaround was added.
