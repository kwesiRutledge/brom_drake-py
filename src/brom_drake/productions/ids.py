from enum import Enum, unique

@unique
class ProductionID(Enum):
    kNotDefined = "NotDefined"
    kShowMeThisModel = "ShowMeThisModel"
    kShelfPlanning1 = "ShelfPlanning1"
    kChemLab1 = "ChemLab-Planning1"
    kChemLab2 = "ChemLab-Planning2"
    kChemLab3 = "ChemLab-Planning3"
    kDemonstrateStaticGrasp = "Demonstrate-StaticGrasp"