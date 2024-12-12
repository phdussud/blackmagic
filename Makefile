all:
	@echo "Makefile build system is depricated."
	@echo ""
	@echo "Please use Meson build system instead."
	@echo ""
	@echo "For most basic build, which builds only Black Magic Debug Application (BMDA), run the following commands:"
	@echo "> meson setup build"
	@echo "> meson compile -C build"
	@echo ""
	@echo "You can find example firmware configuration files in the \`cross-file\` subdirectory."
	@echo "For example, to build a firmware for the native hardware run the following commands:"
	@echo "> meson setup build-native --cross-file cross-file/native.ini --werror"
	@echo "> meson compile -C build-native"
	@echo ""
	@echo "For further instructions please refer to the README.md in the root directory of this repository."
	@echo ""
	@exit 1
