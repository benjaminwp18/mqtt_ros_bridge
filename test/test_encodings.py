from test_msgs.msg import (Arrays, BasicTypes, BoundedPlainSequences, BoundedSequences, Builtins,
                           Constants, Defaults, Empty, MultiNested, Strings, UnboundedSequences,
                           WStrings)
from mqtt_ros_bridge.encodings import human_readable_encoding, human_readable_decoding, MsgLikeT


def encodings_helper(msg: MsgLikeT) -> MsgLikeT:
    assert human_readable_decoding(human_readable_encoding(msg), type(MsgLikeT)) == msg


def test_encodings() -> None:
    encodings_helper(Arrays())
    encodings_helper(BasicTypes())
    encodings_helper(BoundedPlainSequences())
    encodings_helper(BoundedSequences())
    encodings_helper(Builtins())
    encodings_helper(Constants())
    encodings_helper(Defaults())
    encodings_helper(Empty())
    encodings_helper(MultiNested())
    encodings_helper(Strings())
    encodings_helper(UnboundedSequences())
    encodings_helper(WStrings())
