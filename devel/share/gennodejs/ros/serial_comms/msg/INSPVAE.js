// Auto-generated. Do not edit!

// (in-package serial_comms.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class INSPVAE {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.week = null;
      this.seconds = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude = null;
      this.undulation = null;
      this.std_lat = null;
      this.std_lon = null;
      this.std_alt = null;
      this.ve = null;
      this.vn = null;
      this.vu = null;
      this.std_ve = null;
      this.std_vn = null;
      this.std_vu = null;
      this.pitch = null;
      this.roll = null;
      this.yaw = null;
      this.std_pitch = null;
      this.std_roll = null;
      this.std_yaw = null;
      this.ns = null;
      this.gnss_st = null;
      this.nav_st = null;
      this.odo_st = null;
      this.nav_status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('week')) {
        this.week = initObj.week
      }
      else {
        this.week = 0;
      }
      if (initObj.hasOwnProperty('seconds')) {
        this.seconds = initObj.seconds
      }
      else {
        this.seconds = 0.0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('undulation')) {
        this.undulation = initObj.undulation
      }
      else {
        this.undulation = 0.0;
      }
      if (initObj.hasOwnProperty('std_lat')) {
        this.std_lat = initObj.std_lat
      }
      else {
        this.std_lat = 0.0;
      }
      if (initObj.hasOwnProperty('std_lon')) {
        this.std_lon = initObj.std_lon
      }
      else {
        this.std_lon = 0.0;
      }
      if (initObj.hasOwnProperty('std_alt')) {
        this.std_alt = initObj.std_alt
      }
      else {
        this.std_alt = 0.0;
      }
      if (initObj.hasOwnProperty('ve')) {
        this.ve = initObj.ve
      }
      else {
        this.ve = 0.0;
      }
      if (initObj.hasOwnProperty('vn')) {
        this.vn = initObj.vn
      }
      else {
        this.vn = 0.0;
      }
      if (initObj.hasOwnProperty('vu')) {
        this.vu = initObj.vu
      }
      else {
        this.vu = 0.0;
      }
      if (initObj.hasOwnProperty('std_ve')) {
        this.std_ve = initObj.std_ve
      }
      else {
        this.std_ve = 0.0;
      }
      if (initObj.hasOwnProperty('std_vn')) {
        this.std_vn = initObj.std_vn
      }
      else {
        this.std_vn = 0.0;
      }
      if (initObj.hasOwnProperty('std_vu')) {
        this.std_vu = initObj.std_vu
      }
      else {
        this.std_vu = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('std_pitch')) {
        this.std_pitch = initObj.std_pitch
      }
      else {
        this.std_pitch = 0.0;
      }
      if (initObj.hasOwnProperty('std_roll')) {
        this.std_roll = initObj.std_roll
      }
      else {
        this.std_roll = 0.0;
      }
      if (initObj.hasOwnProperty('std_yaw')) {
        this.std_yaw = initObj.std_yaw
      }
      else {
        this.std_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('ns')) {
        this.ns = initObj.ns
      }
      else {
        this.ns = 0;
      }
      if (initObj.hasOwnProperty('gnss_st')) {
        this.gnss_st = initObj.gnss_st
      }
      else {
        this.gnss_st = 0;
      }
      if (initObj.hasOwnProperty('nav_st')) {
        this.nav_st = initObj.nav_st
      }
      else {
        this.nav_st = 0;
      }
      if (initObj.hasOwnProperty('odo_st')) {
        this.odo_st = initObj.odo_st
      }
      else {
        this.odo_st = 0;
      }
      if (initObj.hasOwnProperty('nav_status')) {
        this.nav_status = initObj.nav_status
      }
      else {
        this.nav_status = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type INSPVAE
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [week]
    bufferOffset = _serializer.uint32(obj.week, buffer, bufferOffset);
    // Serialize message field [seconds]
    bufferOffset = _serializer.float64(obj.seconds, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float64(obj.altitude, buffer, bufferOffset);
    // Serialize message field [undulation]
    bufferOffset = _serializer.float64(obj.undulation, buffer, bufferOffset);
    // Serialize message field [std_lat]
    bufferOffset = _serializer.float64(obj.std_lat, buffer, bufferOffset);
    // Serialize message field [std_lon]
    bufferOffset = _serializer.float64(obj.std_lon, buffer, bufferOffset);
    // Serialize message field [std_alt]
    bufferOffset = _serializer.float64(obj.std_alt, buffer, bufferOffset);
    // Serialize message field [ve]
    bufferOffset = _serializer.float64(obj.ve, buffer, bufferOffset);
    // Serialize message field [vn]
    bufferOffset = _serializer.float64(obj.vn, buffer, bufferOffset);
    // Serialize message field [vu]
    bufferOffset = _serializer.float64(obj.vu, buffer, bufferOffset);
    // Serialize message field [std_ve]
    bufferOffset = _serializer.float64(obj.std_ve, buffer, bufferOffset);
    // Serialize message field [std_vn]
    bufferOffset = _serializer.float64(obj.std_vn, buffer, bufferOffset);
    // Serialize message field [std_vu]
    bufferOffset = _serializer.float64(obj.std_vu, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float64(obj.pitch, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float64(obj.roll, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [std_pitch]
    bufferOffset = _serializer.float64(obj.std_pitch, buffer, bufferOffset);
    // Serialize message field [std_roll]
    bufferOffset = _serializer.float64(obj.std_roll, buffer, bufferOffset);
    // Serialize message field [std_yaw]
    bufferOffset = _serializer.float64(obj.std_yaw, buffer, bufferOffset);
    // Serialize message field [ns]
    bufferOffset = _serializer.uint32(obj.ns, buffer, bufferOffset);
    // Serialize message field [gnss_st]
    bufferOffset = _serializer.uint32(obj.gnss_st, buffer, bufferOffset);
    // Serialize message field [nav_st]
    bufferOffset = _serializer.uint32(obj.nav_st, buffer, bufferOffset);
    // Serialize message field [odo_st]
    bufferOffset = _serializer.uint32(obj.odo_st, buffer, bufferOffset);
    // Serialize message field [nav_status]
    bufferOffset = _serializer.string(obj.nav_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type INSPVAE
    let len;
    let data = new INSPVAE(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [week]
    data.week = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [seconds]
    data.seconds = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [undulation]
    data.undulation = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [std_lat]
    data.std_lat = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [std_lon]
    data.std_lon = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [std_alt]
    data.std_alt = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ve]
    data.ve = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vn]
    data.vn = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vu]
    data.vu = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [std_ve]
    data.std_ve = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [std_vn]
    data.std_vn = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [std_vu]
    data.std_vu = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [std_pitch]
    data.std_pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [std_roll]
    data.std_roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [std_yaw]
    data.std_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ns]
    data.ns = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gnss_st]
    data.gnss_st = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [nav_st]
    data.nav_st = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [odo_st]
    data.odo_st = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [nav_status]
    data.nav_status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.nav_status);
    return length + 184;
  }

  static datatype() {
    // Returns string type for a message object
    return 'serial_comms/INSPVAE';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '952992b8f2d9974237c40c1e9c3b19ce';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint32 week
    float64 seconds
    float64 latitude
    float64 longitude
    float64 altitude
    float64 undulation
    float64 std_lat
    float64 std_lon
    float64 std_alt
    float64 ve
    float64 vn
    float64 vu
    float64 std_ve
    float64 std_vn
    float64 std_vu
    float64 pitch
    float64 roll
    float64 yaw
    float64 std_pitch
    float64 std_roll
    float64 std_yaw
    uint32 ns
    uint32 gnss_st
    uint32 nav_st
    uint32 odo_st
    string nav_status
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new INSPVAE(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.week !== undefined) {
      resolved.week = msg.week;
    }
    else {
      resolved.week = 0
    }

    if (msg.seconds !== undefined) {
      resolved.seconds = msg.seconds;
    }
    else {
      resolved.seconds = 0.0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.undulation !== undefined) {
      resolved.undulation = msg.undulation;
    }
    else {
      resolved.undulation = 0.0
    }

    if (msg.std_lat !== undefined) {
      resolved.std_lat = msg.std_lat;
    }
    else {
      resolved.std_lat = 0.0
    }

    if (msg.std_lon !== undefined) {
      resolved.std_lon = msg.std_lon;
    }
    else {
      resolved.std_lon = 0.0
    }

    if (msg.std_alt !== undefined) {
      resolved.std_alt = msg.std_alt;
    }
    else {
      resolved.std_alt = 0.0
    }

    if (msg.ve !== undefined) {
      resolved.ve = msg.ve;
    }
    else {
      resolved.ve = 0.0
    }

    if (msg.vn !== undefined) {
      resolved.vn = msg.vn;
    }
    else {
      resolved.vn = 0.0
    }

    if (msg.vu !== undefined) {
      resolved.vu = msg.vu;
    }
    else {
      resolved.vu = 0.0
    }

    if (msg.std_ve !== undefined) {
      resolved.std_ve = msg.std_ve;
    }
    else {
      resolved.std_ve = 0.0
    }

    if (msg.std_vn !== undefined) {
      resolved.std_vn = msg.std_vn;
    }
    else {
      resolved.std_vn = 0.0
    }

    if (msg.std_vu !== undefined) {
      resolved.std_vu = msg.std_vu;
    }
    else {
      resolved.std_vu = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.std_pitch !== undefined) {
      resolved.std_pitch = msg.std_pitch;
    }
    else {
      resolved.std_pitch = 0.0
    }

    if (msg.std_roll !== undefined) {
      resolved.std_roll = msg.std_roll;
    }
    else {
      resolved.std_roll = 0.0
    }

    if (msg.std_yaw !== undefined) {
      resolved.std_yaw = msg.std_yaw;
    }
    else {
      resolved.std_yaw = 0.0
    }

    if (msg.ns !== undefined) {
      resolved.ns = msg.ns;
    }
    else {
      resolved.ns = 0
    }

    if (msg.gnss_st !== undefined) {
      resolved.gnss_st = msg.gnss_st;
    }
    else {
      resolved.gnss_st = 0
    }

    if (msg.nav_st !== undefined) {
      resolved.nav_st = msg.nav_st;
    }
    else {
      resolved.nav_st = 0
    }

    if (msg.odo_st !== undefined) {
      resolved.odo_st = msg.odo_st;
    }
    else {
      resolved.odo_st = 0
    }

    if (msg.nav_status !== undefined) {
      resolved.nav_status = msg.nav_status;
    }
    else {
      resolved.nav_status = ''
    }

    return resolved;
    }
};

module.exports = INSPVAE;
