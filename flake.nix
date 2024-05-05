{
  description = "STM32 emulator";

  inputs = {
    devenv-root = {
      url = "file+file:///dev/null";
      flake = false;
    };
    nixpkgs.url = "github:cachix/devenv-nixpkgs/rolling";
    devenv.url = "github:cachix/devenv";
    nix2container.url = "github:nlewo/nix2container";
    nix2container.inputs.nixpkgs.follows = "nixpkgs";
    mk-shell-bin.url = "github:rrbutani/nix-mk-shell-bin";
  };

  outputs = inputs @ {
    flake-parts,
    devenv-root,
    ...
  }:
    flake-parts.lib.mkFlake {inherit inputs;} {
      imports = [
        inputs.devenv.flakeModule
        inputs.flake-parts.flakeModules.easyOverlay
      ];
      systems = ["x86_64-linux" "x86_64-darwin" "aarch64-linux" "aarch64-darwin"];

      perSystem = { config, self', inputs', pkgs, system, ... }: let
        defaultCrateOverrides = pkgs.defaultCrateOverrides // {
          stm32-emulator = _: {
            buildInputs = [pkgs.SDL2];
          };
          capstone-sys = _: {
            buildInputs = [pkgs.capstone];
          };
          unicorn-engine = _: {
            buildInputs = [pkgs.unicorn-emu];
            nativeBuildInputs = [pkgs.cmake pkgs.pkg-config];
          };
          sdl2-sys = _: {
            buildInputs = [pkgs.SDL2];
            nativeBuildInputs = [pkgs.cmake];
          };
        };
      in {
        packages = let
          cargoNix = pkgs.callPackage ./Cargo.nix {inherit pkgs defaultCrateOverrides;};
        in rec {
          stm32-emulator = cargoNix.rootCrate.build;
          default = stm32-emulator;
        };

        overlayAttrs = {
          inherit (config.packages) stm32-emulator;
        };

        devenv.shells.default = {
          devenv.root = let
            devenvRootFileContent = builtins.readFile devenv-root.outPath;
          in
            pkgs.lib.mkIf (devenvRootFileContent != "") devenvRootFileContent;

          name = "my-project";

          packages = [config.packages.default];

          enterShell = ''
          '';
        };
      };
      flake = {
        # The usual flake attributes can be defined here, including system-
        # agnostic ones like nixosModule and system-enumerating ones, although
        # those are more easily expressed in perSystem.
      };
    };

  nixConfig = {
    extra-trusted-public-keys = "devenv.cachix.org-1:w1cLUi8dv3hnoSPGAuibQv+f9TZLr6cv/Hm9XgU50cw=";
    extra-substituters = "https://devenv.cachix.org";
  };

}
