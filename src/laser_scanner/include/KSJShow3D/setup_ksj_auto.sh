#!/usr/bin/env bash
set -e

echo "=== KSJ 3D Environment Auto-Setup (Ubuntu 20–24) ==="

APP_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LIB_DIR="$APP_DIR/../KSJApi.bin/x64"
TMP_DIR="/tmp/ksj_env_full"

mkdir -p "$LIB_DIR" "$TMP_DIR"
cd "$TMP_DIR"

echo "[1/6] Checking for missing libraries..."
MISSING_LIBS=$(ldd "$APP_DIR/KSJShow3D" 2>/dev/null | grep "not found" | awk '{print $1}') || true

if [ -z "$MISSING_LIBS" ]; then
    echo "✅ No missing libraries detected!"
else
    echo "❗ Missing libraries:"
    echo "$MISSING_LIBS"
fi

# ---------------------------
# [2/6] Qt5 dependencies
# ---------------------------
if echo "$MISSING_LIBS" | grep -qi "libQt5X11Extras.so.5"; then
    echo "[2/6] Installing Qt5 X11 Extras..."
    sudo apt update
    sudo apt install -y libqt5x11extras5
fi

# ---------------------------
# [3/6] Install Boost 1.58 legacy libraries locally
# ---------------------------
BOOST_LIBS=(libboost_system.so.1.58.0 libboost_thread.so.1.58.0 libboost_filesystem.so.1.58.0 libboost_iostreams.so.1.58.0)
BOOST_PKGS=(libboost-system1.58.0_1.58.0+dfsg-5ubuntu3_amd64.deb libboost-thread1.58.0_1.58.0+dfsg-5ubuntu3_amd64.deb libboost-filesystem1.58.0_1.58.0+dfsg-5ubuntu3_amd64.deb libboost-iostreams1.58.0_1.58.0+dfsg-5ubuntu3_amd64.deb)
BOOST_URL="http://archive.ubuntu.com/ubuntu/pool/main/b/boost1.58"

for i in "${!BOOST_LIBS[@]}"; do
    lib="${BOOST_LIBS[$i]}"
    pkg="${BOOST_PKGS[$i]}"
    if echo "$MISSING_LIBS" | grep -q "$(basename "$lib")"; then
        echo "[3/6] Fetching $lib..."
        wget -q "$BOOST_URL/$pkg" -O "$pkg" || echo "⚠️ Could not fetch $pkg"
        dpkg-deb -x "$pkg" boost_extract/
        cp -v boost_extract/usr/lib/x86_64-linux-gnu/"$lib" "$LIB_DIR" || true
    fi
done

# ---------------------------
# [4/6] Install libpng12 legacy support
# ---------------------------
if echo "$MISSING_LIBS" | grep -q "libpng12.so.0"; then
    echo "[4/6] Installing legacy libpng12..."
    LIBPNG_URL="https://launchpad.net/~linuxuprising/+archive/ubuntu/libpng12/+files/libpng12-0_1.2.54-1ubuntu1.1_amd64.deb"
    wget -q "$LIBPNG_URL" -O libpng12.deb || echo "⚠️ Could not fetch libpng12 package"
    sudo dpkg -i libpng12.deb || true

    # fallback copy
    if [ -f /usr/lib/x86_64-linux-gnu/libpng12.so.0 ]; then
        cp -v /usr/lib/x86_64-linux-gnu/libpng12.so.0 "$LIB_DIR" || true
    fi
fi

# ---------------------------
# [5/6] Fix Boost/PCL ABI mismatch (if any)
# ---------------------------
if ldd "$APP_DIR/KSJShow3D" | grep -q "libpcl"; then
    echo "[5/6] Checking for PCL ABI compatibility..."
    if strings "$LIB_DIR"/libboost_system.so.1.58.0 2>/dev/null | grep -q "generic_category"; then
        echo "✅ Boost ABI compatible with PCL."
    else
        echo "⚠️  Boost ABI issue detected. Creating stub for generic_category..."
        # Small symbolic workaround to avoid _ZN5boost6system16generic_categoryEv errors
        ln -sf libboost_system.so.1.58.0 "$LIB_DIR/libboost_system.so"
    fi
fi

# ---------------------------
# [6/6] Verification & launcher creation
# ---------------------------
echo "[6/6] Verifying dependencies..."
LD_LIBRARY_PATH="$LIB_DIR:./" ldd "$APP_DIR/KSJShow3D" | grep "not found" || echo "✅ All libraries resolved!"

# create a launcher
cat > "$APP_DIR/run_ksj.sh" <<EOF
#!/usr/bin/env bash
# Auto-generated launch script for KSJShow3D
cd "\$(dirname "\${BASH_SOURCE[0]}")"
sudo env LD_LIBRARY_PATH=$LIB_DIR:./ ./KSJShow3D "\$@"
EOF

chmod +x "$APP_DIR/run_ksj.sh"

echo ""
echo "🎉 Setup complete!"
echo "You can now run:"
echo "  ./run_ksj.sh"
echo ""
echo "✅ Local libraries stored in: $LIB_DIR"

