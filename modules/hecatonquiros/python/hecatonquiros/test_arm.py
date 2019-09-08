
import IPython

from ModelSolverOR import ModelSolverOR

data = {}
data["visualize"] = True
data["robot_file"]= "/home/bardo-reborn/programming/hecatonquiros/modules/hecatonquiros/config/single_arm_manipulator_4dof.env.xml"
ms = ModelSolverOR(data)

targetPos = [0.1, 0.2, 0.1]

res, solution = ms.checkIk(targetPos)

IPython.embed()