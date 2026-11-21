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
from brom_drake.watchers.diagram_watcher.diagram_watcher_options import (
    DiagramWatcherOptions,
)
from brom_drake.watchers.port_watcher.port_watcher_options import FigureNamingConvention


class ProductionWatcherOptionsTest(unittest.TestCase):
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
