{
  description = "Pendulum development shell";

  inputs = {
    nixpkgs.url = "nixpkgs";

    rust-src.url = "https://github.com/esp-rs/rust-build/releases/download/v1.93.0.0/rust-src-1.93.0.0.tar.xz";
    rust-src.flake = false;

    xtensa-rust-aarch64-darwin.url = "https://github.com/esp-rs/rust-build/releases/download/v1.93.0.0/rust-1.93.0.0-aarch64-apple-darwin.tar.xz";
    xtensa-rust-aarch64-darwin.flake = false;

    xtensa-rust-aarch64-linux.url = "https://github.com/esp-rs/rust-build/releases/download/v1.93.0.0/rust-1.93.0.0-aarch64-unknown-linux-gnu.tar.xz";
    xtensa-rust-aarch64-linux.flake = false;

    xtensa-rust-x86_64-linux.url = "https://github.com/esp-rs/rust-build/releases/download/v1.93.0.0/rust-1.93.0.0-x86_64-unknown-linux-gnu.tar.xz";
    xtensa-rust-x86_64-linux.flake = false;

    xtensa-gcc-aarch64-darwin.url = "https://github.com/espressif/crosstool-NG/releases/download/esp-15.2.0_20250920/xtensa-esp-elf-15.2.0_20250920-aarch64-apple-darwin.tar.xz";
    xtensa-gcc-aarch64-darwin.flake = false;

    xtensa-gcc-aarch64-linux.url = "https://github.com/espressif/crosstool-NG/releases/download/esp-15.2.0_20250920/xtensa-esp-elf-15.2.0_20250920-aarch64-linux-gnu.tar.xz";
    xtensa-gcc-aarch64-linux.flake = false;

    xtensa-gcc-x86_64-linux.url = "https://github.com/espressif/crosstool-NG/releases/download/esp-15.2.0_20250920/xtensa-esp-elf-15.2.0_20250920-x86_64-linux-gnu.tar.xz";
    xtensa-gcc-x86_64-linux.flake = false;
  };

  outputs = inputs@{ self, nixpkgs, ... }:
    let
      lib = nixpkgs.lib;
      systems = [
        "aarch64-darwin"
        "aarch64-linux"
        "x86_64-linux"
      ];

      forAllSystems = f:
        lib.genAttrs systems (system:
          f system (import nixpkgs { inherit system; }));

      rustVersion = "1.93.0.0";
      gccVersion = "15.2.0_20250920";

      rustHostForSystem = {
        aarch64-darwin = "aarch64-apple-darwin";
        aarch64-linux = "aarch64-unknown-linux-gnu";
        x86_64-linux = "x86_64-unknown-linux-gnu";
      };

      rustInputFor = system: inputs.${"xtensa-rust-" + system};
      gccInputFor = system: inputs.${"xtensa-gcc-" + system};
    in
    {
      packages = forAllSystems (system: pkgs:
        let
          rustHost = lib.getAttr system rustHostForSystem;

          xtensaRust = pkgs.stdenvNoCC.mkDerivation {
            pname = "xtensa-rust";
            version = rustVersion;

            src = rustInputFor system;
            rustSrc = inputs.rust-src;

            nativeBuildInputs = [
              pkgs.bash
            ];

            dontUnpack = true;

            installPhase = ''
              runHook preInstall

              bash "$src/install.sh" \
                --destdir="$out" \
                --prefix= \
                --without=rust-docs-json-preview,rust-docs \
                --disable-ldconfig

              chmod -R u+w "$out"

              bash "$rustSrc/install.sh" \
                --destdir="$out" \
                --prefix= \
                --disable-ldconfig

              runHook postInstall
            '';
          };

          xtensaGcc = pkgs.stdenvNoCC.mkDerivation {
            pname = "xtensa-esp-elf";
            version = gccVersion;

            src = gccInputFor system;

            dontUnpack = true;

            installPhase = ''
              runHook preInstall

              mkdir -p "$out"
              cp -R "$src"/. "$out"/

              runHook postInstall
            '';
          };
        in
        {
          xtensa-gcc = xtensaGcc;
          xtensa-rust = xtensaRust;
        });

      devShells = forAllSystems (system: pkgs:
        let
          toolchains = self.packages.${system};
        in
        {
          default = pkgs.mkShell {
            packages = [
              pkgs.espflash
              pkgs.just
              pkgs.ldproxy
              toolchains.xtensa-gcc
              toolchains.xtensa-rust
            ];

            shellHook = ''
              export PS1="\[\e[36m\]❄ pendulum\[\e[0m\] $PS1"
            '';
          };
        });
    };
}
