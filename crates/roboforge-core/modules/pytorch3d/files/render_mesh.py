"""Render an icosphere with PyTorch3D and take a gradient through the image."""
from pathlib import Path

import torch
from pytorch3d.renderer import (
    FoVPerspectiveCameras, MeshRasterizer, MeshRenderer, PointLights,
    RasterizationSettings, SoftPhongShader, TexturesVertex, look_at_view_transform,
)
from pytorch3d.utils import ico_sphere


def main():
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    mesh = ico_sphere(3, device)
    verts = mesh.verts_padded().clone().requires_grad_(True)
    mesh = mesh.update_padded(verts)
    mesh.textures = TexturesVertex(verts_features=torch.full_like(verts, 0.7))

    R, T = look_at_view_transform(dist=2.7, elev=20, azim=30)
    cameras = FoVPerspectiveCameras(device=device, R=R, T=T)
    renderer = MeshRenderer(
        rasterizer=MeshRasterizer(cameras=cameras, raster_settings=RasterizationSettings(image_size=256)),
        shader=SoftPhongShader(device=device, cameras=cameras, lights=PointLights(device=device, location=[[2.0, 2.0, 2.0]])),
    )
    image = renderer(mesh)  # (1, H, W, 4)
    image[..., :3].mean().backward()
    print(f'rendered {tuple(image.shape)}, grad norm on vertices: {verts.grad.norm().item():.4e}')

    out = Path('outputs')
    out.mkdir(exist_ok=True)
    try:
        from torchvision.utils import save_image
        save_image(image[0, ..., :3].permute(2, 0, 1).detach().cpu(), out / 'render.png')
        print(f'wrote {out / "render.png"}')
    except ImportError:
        pass


if __name__ == '__main__':
    main()
