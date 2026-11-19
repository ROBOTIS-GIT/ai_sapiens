#!/usr/bin/env bash
set -euo pipefail

version="${1:-1.23.2}"
case "$(dpkg --print-architecture)" in
  amd64) arch=x64 ;;
  arm64) arch=arm64 ;;
  *) echo "ONNX Runtime: unsupported architecture $(dpkg --print-architecture)" >&2; exit 1 ;;
esac

tmp_dir="$(mktemp -d)"
trap 'rm -rf "$tmp_dir"' EXIT
package="$tmp_dir/onnxruntime.nupkg"
package_url="https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime"
curl --fail --silent --show-error --location --retry 3 \
  "$package_url/$version/microsoft.ml.onnxruntime.$version.nupkg" --output "$package"

if [[ "$version" == 1.23.2 ]]; then
  printf '%s  %s\n' 25fe172f7fcdf34f2b5b02c8b997f6b88abc282956849d4af4335af23d0c4e4a "$package" \
    | sha256sum --check
fi

native_dir="runtimes/linux-$arch/native"
unzip -q "$package" 'build/native/include/*' "$native_dir/libonnxruntime*.so" -d "$tmp_dir"
install -d /usr/local/include/onnxruntime /usr/local/lib
install -m 644 "$tmp_dir"/build/native/include/*.h /usr/local/include/onnxruntime/
install -m 644 "$tmp_dir/$native_dir"/libonnxruntime*.so /usr/local/lib/
ldconfig
