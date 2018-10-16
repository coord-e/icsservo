
class IcsServo {
  using ServoID = std::uint8_t;
  using Position = double;

  void set_position(ServoID, Position);
  void set_free(ServoID);

  void set_stretch(ServoID, std::uint8_t stretch);
  void set_speed(ServoID, std::uint8_t speed);
  void set_current_limit(ServoID, std::uint8_t current_limit);
  void set_temperature_limit(ServoID, std::uint8_t tmperature_limit);

  std::uint8_t get_stretch(ServoID id);
  std::uint8_t get_speed(ServoID id);
  std::uint8_t get_current(ServoID id);
  std::uint8_t get_temperature(ServoID id);
  Position get_position(ServoID id);
};
