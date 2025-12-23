from enum import Enum, unique

@unique
class ProductionID(Enum):
    """
    *Description*

    A unique ID assigned to each production type.

    Options are:

    - `ProductionID.kNotDefined`: Undefined Production ID (Used to Detect Errors in Production Construction)
    - `ProductionID.kShowMeThisModel`
    - `ProductionID.kShelfPlanning1`
    - `ProductionID.kChemLab1`

    TODO(Kwesi): Investigate if there is a way to automatically list all possible values
    in Sphinx.
    """
    kNotDefined = "NotDefined"
    kShowMeThisModel = "ShowMeThisModel"
    kShelfPlanning1 = "ShelfPlanning1"
    kChemLab1 = "ChemLab-Planning1"
    kChemLab2 = "ChemLab-Planning2"
    kChemLab3 = "ChemLab-Planning3"
    kDemonstrateStaticGrasp = "Demonstrate-StaticGrasp"
    kAttemptGraspWithPuppeteer = "Attempt-Grasp-WithPuppeteer"
    kAttemptGraspWithStaticWrist = "Attempt-Grasp-WithStaticWrist"