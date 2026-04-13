from opendbc.can import CANPacker, CANParser
from opendbc.car.ford.fordcan import CanBus, create_apa_command


def test_create_apa_command_roundtrip():
  packer = CANPacker("ford_lincoln_base_pt")
  CAN = CanBus(fingerprint={})  # default: main=0, radar=1, camera=2

  addr, data, bus = create_apa_command(
    packer, CAN, apply_angle=42.5, angle_req=True,
    sapp_config=86, sapp_action=2, sapp_chime=0,
  )

  assert bus == CAN.camera == 2
  assert addr == 0x3A8

  msgs = [("ParkAid_Data", 0)]
  parser = CANParser("ford_lincoln_base_pt", msgs, bus)
  parser.update([(bus, [(addr, data, bus)])])

  vals = parser.vl["ParkAid_Data"]
  assert abs(vals["ExtSteeringAngleReq2"] - 42.5) < 0.1
  assert vals["EPASExtAngleStatReq"] == 1
  assert vals["SAPPStatusCoding"] == 86
  assert vals["ApaSys_D_Stat"] == 2
  assert vals["ApaChime_D_Rq"] == 0


def test_create_apa_command_disabled():
  packer = CANPacker("ford_lincoln_base_pt")
  CAN = CanBus(fingerprint={})

  addr, data, bus = create_apa_command(
    packer, CAN, apply_angle=0.0, angle_req=False,
    sapp_config=0, sapp_action=0,
  )

  assert addr == 0x3A8
  assert bus == CAN.camera

  msgs = [("ParkAid_Data", 0)]
  parser = CANParser("ford_lincoln_base_pt", msgs, bus)
  parser.update([(bus, [(addr, data, bus)])])

  vals = parser.vl["ParkAid_Data"]
  assert abs(vals["ExtSteeringAngleReq2"] - 0.0) < 0.1
  assert vals["EPASExtAngleStatReq"] == 0
  assert vals["SAPPStatusCoding"] == 0
  assert vals["ApaSys_D_Stat"] == 0
  assert vals["ApaChime_D_Rq"] == 0
