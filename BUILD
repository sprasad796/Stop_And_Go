load("@//tools/rules:py.bzl", "atg_py_library", "atg_py_test")

atg_py_library(
    name = "stop_sim",
    srcs = [
        "__init__.py",
        "stop_and_go_actors.py",
        "stop_and_go_check_collision.py",
        "stop_and_go_cord_transform.py",
        "stop_and_go_data.py",
        "stop_and_go_data_generation.py",
        "stop_and_go_data_type.py",
        "stop_and_go_dataset_schema.py",
        "stop_and_go_draw.py",
        "stop_and_go_globals.py",
        "stop_and_go_intersection_rules.py",
        "stop_and_go_main.py",
        "stop_and_go_main_loop.py",
        "stop_and_go_rotate_image.py",
        "stop_and_go_sim.py",
        "stop_and_go_subimage.py",
        "stop_and_go_view.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@pypi_pygame",
    ],
)

atg_py_test(
    name = "stop_sim_test",
    size = "small",
    srcs = glob(
        [
            "test/*.py",
        ],
    ),
    data = [
        "stop_and_go_config.yml",
        "stop_and_go_dataset_config.yml",
    ],
    deps = [
	"@pypi_pygame",
    ],
)
