import unittest
from unittest.mock import MagicMock, patch

from brom_drake.productions.debug.grasping.attempt_grasp.with_static_wrist.production import (
    AttemptGraspWithStaticWrist,
)
from brom_drake.productions.debug.grasping.show_me_this_static_grasp.production import (
    ShowMeThisStaticGrasp,
)
from brom_drake.productions.debug.show_me.show_me_this_model import ShowMeThisModel
from brom_drake.productions.types.base.base import BaseProduction
from brom_drake.productions.types.motion_planning.offline import dynamic, kinematic
from brom_drake.productions.types.pick_and_place import mp_and_grasping
from brom_drake.motion_planning.systems.async_planner import AsyncPlanningConfig
from brom_drake.productions.motion_planning.offline.kinematic.chem_lab1 import ChemLab1
from brom_drake.productions.motion_planning.offline.kinematic.shelf import (
    ShelfPlanning1,
)
from brom_drake.productions.motion_planning.offline.dynamic.chem_lab2 import ChemLab2
from brom_drake.productions.motion_planning.offline.dynamic.cornell_tech1 import (
    CornellTech1,
)
from brom_drake.productions.pick_and_place.chem_lab3 import ChemLab3
from brom_drake.utils.collision_checking import CollisionCheckMethod
from brom_drake.watchers.diagram_watcher.diagram_watcher_options import (
    DiagramWatcherOptions,
)
from brom_drake.watchers.port_watcher.port_watcher_options import FigureNamingConvention


class ProductionWatcherOptionsTest(unittest.TestCase):
    def test_easy_build_forwards_collision_settings(self):
        for module, production_type in (
            (kinematic, kinematic.KinematicMotionPlanningProduction),
            (dynamic, dynamic.OfflineDynamicMotionPlanningProduction),
            (mp_and_grasping, mp_and_grasping.MotionPlanningAndGraspingProduction),
        ):
            async_options = (
                (None, AsyncPlanningConfig()) if module != mp_and_grasping else (None,)
            )
            for async_config in async_options:
                for settings in (
                    {},
                    {
                        "collision_threshold": 0.001,
                        "collision_check_method": CollisionCheckMethod.SIGNED_DISTANCE,
                    },
                ):
                    with self.subTest(
                        production=production_type,
                        async_config=async_config,
                        settings=settings,
                    ):
                        production = MagicMock()
                        production.suggested_roles = [MagicMock()]
                        production.build_production.return_value = (
                            "diagram",
                            "context",
                        )
                        planner_class = (
                            "AsyncPlannerSystem"
                            if async_config
                            else "PrototypicalPlannerSystem"
                        )
                        kwargs = dict(settings)
                        if async_config:
                            kwargs["async_planning"] = async_config
                        with patch.object(module, planner_class) as planner:
                            production_type.easy_cast_and_build(
                                production, MagicMock(), **kwargs
                            )
                        self.assertEqual(
                            planner.call_args.kwargs["collision_threshold"],
                            settings.get("collision_threshold", 0.01),
                        )
                        self.assertEqual(
                            planner.call_args.kwargs["collision_check_method"],
                            settings.get(
                                "collision_check_method",
                                CollisionCheckMethod.PENETRATION,
                            ),
                        )
                        production.fill_role.assert_called_once_with(
                            production.suggested_roles[0], planner.return_value
                        )

    def test_concrete_productions_forward_collision_settings(self):
        for production_type, base_type in (
            (ChemLab1, kinematic.KinematicMotionPlanningProduction),
            (ShelfPlanning1, kinematic.KinematicMotionPlanningProduction),
            (ChemLab2, dynamic.OfflineDynamicMotionPlanningProduction),
            (CornellTech1, dynamic.OfflineDynamicMotionPlanningProduction),
            (ChemLab3, mp_and_grasping.MotionPlanningAndGraspingProduction),
        ):
            with self.subTest(production=production_type):
                production = object.__new__(production_type)
                algorithm = MagicMock()
                with patch.object(
                    base_type, "easy_cast_and_build", side_effect=RuntimeError("stop")
                ) as build:
                    with self.assertRaisesRegex(RuntimeError, "stop"):
                        production.easy_cast_and_build(
                            algorithm,
                            collision_threshold=0.001,
                            collision_check_method=CollisionCheckMethod.SIGNED_DISTANCE,
                        )
                self.assertEqual(build.call_args.args, (algorithm,))
                self.assertEqual(build.call_args.kwargs["collision_threshold"], 0.001)
                self.assertEqual(
                    build.call_args.kwargs["collision_check_method"],
                    CollisionCheckMethod.SIGNED_DISTANCE,
                )

    def test_easy_build_forwards_options(self):
        for module, production_type in (
            (kinematic, kinematic.KinematicMotionPlanningProduction),
            (dynamic, dynamic.OfflineDynamicMotionPlanningProduction),
            (mp_and_grasping, mp_and_grasping.MotionPlanningAndGraspingProduction),
        ):
            for options in (None, DiagramWatcherOptions()):
                for with_watcher in (False, True):
                    with self.subTest(
                        production=production_type.__name__,
                        options=options,
                        with_watcher=with_watcher,
                    ):
                        production = MagicMock()
                        production.suggested_roles = [MagicMock()]
                        production.build_production.return_value = (
                            "diagram",
                            "context",
                        )
                        with patch.object(module, "PrototypicalPlannerSystem"):
                            result = production_type.easy_cast_and_build(
                                production,
                                MagicMock(),
                                with_watcher=with_watcher,
                                watcher_options=options,
                            )
                        production.build_production.assert_called_once_with(
                            with_watcher=with_watcher, watcher_options=options
                        )
                        self.assertEqual(result, ("diagram", "context"))

    def test_debug_build_forwards_options(self):
        for production_type in (
            ShowMeThisModel,
            ShowMeThisStaticGrasp,
            AttemptGraspWithStaticWrist,
        ):
            for options in (None, DiagramWatcherOptions()):
                with self.subTest(production=production_type.__name__, options=options):
                    production = object.__new__(production_type)
                    production.plant = MagicMock()
                    production.show_me_system = MagicMock()
                    production.diagram, production.diagram_context = (
                        "diagram",
                        "context",
                    )
                    with patch.object(
                        BaseProduction, "add_cast_and_build", autospec=True
                    ) as build:
                        result = production.add_cast_and_build(
                            [], watcher_options=options
                        )
                    kwargs = {"watcher_options": options}
                    if production_type is not ShowMeThisModel:
                        kwargs["with_watcher"] = True
                    if production_type is AttemptGraspWithStaticWrist:
                        kwargs["figure_naming_convention"] = (
                            FigureNamingConvention.kHierarchical
                        )
                    build.assert_called_once_with(production, [], **kwargs)
                    self.assertEqual(result, ("diagram", "context"))


if __name__ == "__main__":
    unittest.main()
