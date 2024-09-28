from brom_drake.scenes.types.motion_planning import OfflineMotionPlanningScene


class ShelfPlanningScene(OfflineMotionPlanningScene):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def add_all_secondary_cast_members_to_builder(self, builder):
        pass

    def suggested_roles(self):
        return []

    def fill_role(self, role, system):
        pass