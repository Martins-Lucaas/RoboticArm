from setuptools import find_packages, setup
import os
from glob import glob

package_name = "covvi"

def collect_recursive(src_dir, dest_prefix):
    """Coleta arquivos recursivamente preservando a árvore a partir de src_dir."""
    items = []
    if os.path.isdir(src_dir):
        for dirpath, _, files in os.walk(src_dir):
            if not files:
                continue
            # caminho relativo dentro de src_dir
            rel = os.path.relpath(dirpath, src_dir)
            dest = (
                os.path.join("share", package_name, dest_prefix, rel)
                if rel != "."
                else os.path.join("share", package_name, dest_prefix)
            )
            items.append((dest, [os.path.join(dirpath, f) for f in files]))
    return items

# --- URDF/Xacro do pacote (texto)
urdf_files = [
    (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
    (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
]

# --- Shadow Hand (NOVO) — meshes, thumbnails, SDF, config
shadow_meshes     = collect_recursive("shadow_hand/meshes", "shadow_hand/meshes")
shadow_thumbnails = collect_recursive("shadow_hand/thumbnails", "shadow_hand/thumbnails")
shadow_top = [
    (
        os.path.join("share", package_name, "shadow_hand"),
        [f for f in ["shadow_hand/model.sdf", "shadow_hand/model.config"] if os.path.exists(f)],
    ),
]

# --- Launch / RViz (opcionais)
launch_files = [(os.path.join("share", package_name, "launch"), glob("launch/*.py"))]
rviz_files   = [(os.path.join("share", package_name, "rviz"),   glob("rviz/*.rviz"))]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # índice do ament para o nome do pacote
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ]
    + urdf_files
    + shadow_meshes
    + shadow_thumbnails
    + shadow_top
    + launch_files
    + rviz_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lucas-pc",
    maintainer_email="lucaspmartins14@gmail.com",
    description="CR10 + Shadow Hand (RViz/Webots/ROS2)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
