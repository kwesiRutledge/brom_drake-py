from brom_drake.scenes.types import BaseScene
from brom_drake.scenes.roles import kOfflineMotionPlanner


class OfflineMotionPlanningScene(BaseScene):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def add_all_secondary_cast_members_to_builder(self, builder):
        pass

    def suggested_roles(self):
        return [kOfflineMotionPlanner]