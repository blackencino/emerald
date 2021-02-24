from conans import ConanFile, CMake 

class EmeraldConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"

    options = {
        "enable_guis": [True, False]
    }

    default_options = {
        "enable_guis": False,
    }

    requires = (
        "fmt/7.1.3",
        "boost/1.74.0",
        "cxxopts/2.2.1",
        "gsl-lite/0.37.0",
        "openexr/2.5.3",
        "alembic/1.7.16@blackencino/latest",
        "outcome/2.1.5",
        "spdlog/1.8.2",
        "gtest/1.10.0",
        "tbb/2020.0"
    )

    generators = "cmake_paths", "cmake_find_package"

    def requirements(self):
        self.options["tbb"].tbbmalloc = True
        self.options["tbb"].tbbproxy = False
        self.options["tbb"].shared = False

        if self.options.enable_guis:
            self.requires.add("glfw/3.3.2")
            self.requires.add("imgui/1.79")
            self.requires.add("glad/0.1.34")

            #self.options["glfw"].fPIC = True
            #self.options["imgui"].fPIC = True

            self.options["glad"].shared = False
            #self.options["glad"].fPIC = True
            self.options["glad"].spec = "gl"
            self.options["glad"].no_loader = False
            self.options["glad"].gl_profile = "core"
            self.options["glad"].gl_version = "4.1"

    def imports(self):
        if self.options.enable_guis:
            self.copy("imgui_impl_glfw.*", 
                root_package="imgui*",
                src="./res/bindings",
                dst="imgui_backend")
            self.copy("imgui_impl_opengl3.*",
                root_package="imgui*",
                src="./res/bindings",
                dst="imgui_backend")


