from enum import Enum, unique

@unique
class SceneID(Enum):
    kNotDefined = "NotDefined"
    kShowMeThisModel = "ShowMeThisModel"
    kShelfPlanning1 = "ShelfPlanning1"
    kChemLab1 = "ChemLab-Planning1"