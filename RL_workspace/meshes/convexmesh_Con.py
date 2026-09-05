import trimesh
import vhacdx

# Load original mesh and force single-mesh format
mesh = trimesh.load("base_link.STL", force="mesh")

# Instantiate VHACD or call compute_vhacd
if hasattr(vhacdx, 'VHACD'):
    vhacd = vhacdx.VHACD()
    ch_list = vhacd.compute(mesh.vertices, mesh.faces)
elif hasattr(vhacdx, 'compute_vhacd'):
    ch_list = vhacdx.compute_vhacd(mesh.vertices, mesh.faces)
else:
    # Fallback to direct function call
    ch_list = vhacdx.compute(mesh.vertices, mesh.faces)

# Reconstruct trimesh objects from output vertices and faces
convex_hulls = []
for item in ch_list:
    # Check if returning tuple of (vertices, faces) or object
    if isinstance(item, tuple):
        verts, faces = item[0], item[1]
    else:
        verts, faces = item.vertices, item.faces
    convex_hulls.append(trimesh.Trimesh(verts, faces))

# Combine and export simplified mesh
combined_mesh = trimesh.util.concatenate(convex_hulls)
combined_mesh.export("simple_base_link.stl")
print("Convex decomposition complete! Saved as simple_base_link.stl")
