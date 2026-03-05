from enum import Enum


class ActionType(Enum):
    PROGRESS = 0
    INTERLACE = 1
    WAIT = 2
    NOOP = 3


class ActionDirection(Enum):
    REAR = 0
    FRONT = 1
    NA = 2


def to_type_enum(name: str) -> ActionType:
    return ActionType[name.upper()]


def to_direction_enum(direction: str) -> ActionDirection:
    return ActionDirection[direction.upper()]


def to_arg_values(act_type: ActionType, arg: str):
    match act_type:
        case ActionType.PROGRESS:
            vList = arg.split("-")
            return ActionDirection.NA, int(vList[0]), int(vList[1]), -1
        case ActionType.INTERLACE:
            vList = arg.split("|")
            return to_direction_enum(vList[0]), int(vList[1]), -1, 10
        case ActionType.WAIT:
            return ActionDirection.NA, -1, -1, float(arg)
        case ActionType.NOOP:
            return ActionDirection.NA, -1, -1, -1


class CableAction:
    def __init__(self, type_str: str, arg_str: str):
        self.actionType = to_type_enum(type_str)
        self.actionDirection, self.currVertexId, self.nextVertexId, self.actionDur = (
            to_arg_values(self.actionType, arg_str)
        )

    def __str__(self):
        res = f"{self.actionType.name.lower()}("
        match self.actionDirection:
            case ActionDirection.REAR:
                res += "rear|"
            case ActionDirection.FRONT:
                res += "front|"
            case ActionDirection.NA:
                res += ""
        match self.currVertexId:
            case -1:
                pass
            case _:
                res += str(self.currVertexId)
        match self.nextVertexId:
            case -1:
                pass
            case _:
                res += "-" + str(self.nextVertexId)
        if self.actionType == ActionType.WAIT:
            res += str(self.actionDur)
        res += ")"
        return res

    def __repr__(self):
        res = f"{self.actionType.name.lower()}("
        match self.actionDirection:
            case ActionDirection.REAR:
                res += "rear|"
            case ActionDirection.FRONT:
                res += "front|"
            case ActionDirection.NA:
                res += ""
        match self.currVertexId:
            case -1:
                pass
            case _:
                res += str(self.currVertexId)
        match self.nextVertexId:
            case -1:
                pass
            case _:
                res += "-" + str(self.nextVertexId)
        if self.actionType == ActionType.WAIT:
            res += str(self.actionDur)
        res += ")"
        return res


def assert_all_action_sequences_to_be_same_size(data):
    first_len = len(data[0])
    if not all(len(item) == first_len for item in data[1:]):
        raise AssertionError("Not all action sequences have the same size.")


class ActionSequences:
    def __init__(self, action_sequences_list):
        assert_all_action_sequences_to_be_same_size(action_sequences_list)
        self.actions = action_sequences_list
        self.n_cables = len(self.actions)
        self.n_steps = len(self.actions[0])

    @classmethod
    def from_list(cls, data):
        import re

        pattern = r"(\w+)\((.*?)\)"
        action_sequences = []
        for cable_sequence in data:
            # Process and append to action sequences
            action_list = []
            for action in cable_sequence:
                matches = re.findall(pattern, action.strip())
                for type_str, arg_str in matches:
                    action_list.append(CableAction(type_str, arg_str))
            action_sequences.append(action_list)
        return cls(action_sequences)

    @classmethod
    def from_file(cls, filepath):
        import json

        with open(filepath, "r") as file:
            action_data = json.load(file)

        return cls.from_list(action_data)
