import argparse
import copy
import shutil
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


class MeshResolutionError(RuntimeError):
    pass


def _resolve_mesh_source(mesh_uri: str, urdf_parent: Path) -> Path:
    if mesh_uri.startswith('package://'):
        package_path = mesh_uri[len('package://'):]
        package_name, _, relative_path = package_path.partition('/')
        if not package_name or not relative_path:
            raise MeshResolutionError(f'Invalid package URI: {mesh_uri}')
        share_dir = Path(get_package_share_directory(package_name))
        return share_dir / relative_path

    if mesh_uri.startswith('file://'):
        return Path(mesh_uri[len('file://'):])

    candidate = Path(mesh_uri)
    if candidate.is_absolute():
        return candidate

    return (urdf_parent / candidate).resolve()


def _target_mesh_path(mesh_uri: str, source_path: Path) -> Path:
    if mesh_uri.startswith('package://'):
        package_path = mesh_uri[len('package://'):]
        package_name, _, relative_path = package_path.partition('/')
        return Path('meshes') / package_name / relative_path

    if source_path.is_absolute():
        return Path('meshes') / 'external' / source_path.as_posix().lstrip('/')

    return Path('meshes') / source_path.as_posix()


def _parse_xacro_kv_args(xacro_args: list[str]) -> None:
    for arg in xacro_args:
        if ':=' not in arg:
            raise ValueError(
                f'Invalid --xacro-args entry {arg!r}. Use xacro assignment format name:=value.'
            )


def _expand_xacro(
    input_xacro: Path,
    calibration_file: Path | None,
    kinematics_arg_name: str,
    xacro_args: list[str],
    auto_instantiate_unit: bool,
) -> ET.Element:
    command = ['xacro', str(input_xacro)]

    if auto_instantiate_unit and all(not arg.startswith('instantiate_unit:=') for arg in xacro_args):
        command.append('instantiate_unit:=true')

    if calibration_file is not None:
        command.append(f'{kinematics_arg_name}:={calibration_file}')

    command.extend(xacro_args)

    try:
        result = subprocess.run(command, check=True, text=True, capture_output=True)
    except subprocess.CalledProcessError as error:
        stderr_text = (error.stderr or '').strip()
        stdout_text = (error.stdout or '').strip()
        details = stderr_text or stdout_text or f'xacro exited with code {error.returncode}'
        raise RuntimeError(f'xacro expansion failed for {input_xacro}: {details}') from error

    try:
        return ET.fromstring(result.stdout)
    except ET.ParseError as error:
        raise RuntimeError(f'xacro output is not valid XML for {input_xacro}: {error}') from error


def _calibrated_output_name(base_name: str, calibration_file: Path, calibration_count: int) -> str:
    if calibration_count == 1:
        return base_name

    base_path = Path(base_name)
    stem = base_path.stem
    suffix = base_path.suffix or '.urdf'
    return f'{stem}_{calibration_file.stem}{suffix}'


def export_unit_assets(
    input_urdf: Path | None,
    input_xacro: Path | None,
    output_dir: Path,
    output_urdf_name: str,
    overwrite: bool,
    calibration_files: list[Path],
    kinematics_arg_name: str,
    xacro_args: list[str],
    auto_instantiate_unit: bool,
) -> int:
    if (input_urdf is None) == (input_xacro is None):
        raise ValueError('Specify exactly one of --input-urdf or --input-xacro')

    if input_urdf is not None and calibration_files:
        raise ValueError('Using --calibration-files requires --input-xacro')

    if input_urdf is not None and not input_urdf.exists():
        raise FileNotFoundError(f'Input URDF not found: {input_urdf}')

    if input_xacro is not None and not input_xacro.exists():
        raise FileNotFoundError(f'Input xacro not found: {input_xacro}')

    for calibration_file in calibration_files:
        if not calibration_file.exists():
            raise FileNotFoundError(f'Calibration file not found: {calibration_file}')

    _parse_xacro_kv_args(xacro_args)

    output_dir.mkdir(parents=True, exist_ok=True)
    copied = 0
    resolved_cache = {}

    written = 0
    variants: list[tuple[ET.Element, Path | None, str]] = []

    if input_xacro is not None:
        xacro_parent = input_xacro.resolve().parent
        if calibration_files:
            for calibration_file in calibration_files:
                root = _expand_xacro(
                    input_xacro,
                    calibration_file,
                    kinematics_arg_name,
                    xacro_args,
                    auto_instantiate_unit,
                )
                variants.append((root, calibration_file, str(xacro_parent)))
        else:
            root = _expand_xacro(
                input_xacro,
                None,
                kinematics_arg_name,
                xacro_args,
                auto_instantiate_unit,
            )
            variants.append((root, None, str(xacro_parent)))
    else:
        tree = ET.parse(str(input_urdf))
        variants.append((tree.getroot(), None, str(input_urdf.resolve().parent)))

    for root, calibration_file, urdf_parent_str in variants:
        localized_root = copy.deepcopy(root)
        urdf_parent = Path(urdf_parent_str)

        for mesh in localized_root.findall('.//mesh'):
            filename = mesh.get('filename')
            if not filename:
                continue

            source_path = _resolve_mesh_source(filename, urdf_parent)
            if not source_path.exists():
                raise FileNotFoundError(f'Mesh not found: {filename} -> {source_path}')

            destination_relative = _target_mesh_path(filename, source_path)
            destination_path = output_dir / destination_relative

            cache_key = str(source_path.resolve())
            if cache_key not in resolved_cache:
                destination_path.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source_path, destination_path)
                resolved_cache[cache_key] = destination_relative.as_posix()
                copied += 1

            mesh.set('filename', resolved_cache[cache_key])

        if calibration_file is not None:
            output_name = _calibrated_output_name(output_urdf_name, calibration_file, len(calibration_files))
        else:
            output_name = output_urdf_name

        output_urdf_path = output_dir / output_name
        if output_urdf_path.exists() and not overwrite:
            raise FileExistsError(
                f'Output URDF already exists: {output_urdf_path}. Use --overwrite to replace it.'
            )

        ET.ElementTree(localized_root).write(str(output_urdf_path), encoding='utf-8', xml_declaration=True)
        if calibration_file is None:
            print(f'Exported URDF: {output_urdf_path}')
        else:
            print(f'Exported calibrated URDF: {output_urdf_path} from {calibration_file}')
        written += 1

    print(f'Copied meshes: {copied}')
    print(f'Written URDF files: {written}')
    return copied


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Create a self-contained folder from a URDF/xacro and its meshes.'
    )
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument(
        '--input-urdf',
        type=Path,
        help='Path to expanded URDF file (legacy path, no per-calibration xacro generation).',
    )
    input_group.add_argument(
        '--input-xacro',
        type=Path,
        help='Path to xacro file to expand. Use with --calibration-files for calibrated URDF generation.',
    )
    parser.add_argument(
        '--output-dir',
        required=True,
        type=Path,
        help='Output folder where URDF and meshes will be written.',
    )
    parser.add_argument(
        '--output-urdf-name',
        default='unit.urdf',
        help='Filename for exported URDF inside output dir.',
    )
    parser.add_argument(
        '--calibration-files',
        nargs='+',
        type=Path,
        default=[],
        help=(
            'One or more calibration YAML files. Requires --input-xacro. '
            'A URDF is generated by xacro for each file.'
        ),
    )
    parser.add_argument(
        '--kinematics-arg-name',
        default='kinematics_parameters_file',
        help='Xacro argument name that accepts the calibration YAML path.',
    )
    parser.add_argument(
        '--xacro-args',
        nargs='*',
        default=[],
        help='Extra xacro arguments in name:=value format.',
    )
    parser.add_argument(
        '--no-auto-instantiate-unit',
        action='store_true',
        help='Do not auto-add instantiate_unit:=true when running xacro.',
    )
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='Overwrite output URDF if it already exists.',
    )

    args = parser.parse_args()
    export_unit_assets(
        args.input_urdf,
        args.input_xacro,
        args.output_dir,
        args.output_urdf_name,
        args.overwrite,
        args.calibration_files,
        args.kinematics_arg_name,
        args.xacro_args,
        not args.no_auto_instantiate_unit,
    )


if __name__ == '__main__':
    main()
