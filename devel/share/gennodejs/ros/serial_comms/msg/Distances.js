// Auto-generated. Do not edit!

// (in-package serial_comms.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Distances {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.distance_a = null;
      this.distance_b = null;
      this.distance_c = null;
      this.distance_d = null;
      this.distance_e = null;
      this.distance_f = null;
    }
    else {
      if (initObj.hasOwnProperty('distance_a')) {
        this.distance_a = initObj.distance_a
      }
      else {
        this.distance_a = 0;
      }
      if (initObj.hasOwnProperty('distance_b')) {
        this.distance_b = initObj.distance_b
      }
      else {
        this.distance_b = 0;
      }
      if (initObj.hasOwnProperty('distance_c')) {
        this.distance_c = initObj.distance_c
      }
      else {
        this.distance_c = 0;
      }
      if (initObj.hasOwnProperty('distance_d')) {
        this.distance_d = initObj.distance_d
      }
      else {
        this.distance_d = 0;
      }
      if (initObj.hasOwnProperty('distance_e')) {
        this.distance_e = initObj.distance_e
      }
      else {
        this.distance_e = 0;
      }
      if (initObj.hasOwnProperty('distance_f')) {
        this.distance_f = initObj.distance_f
      }
      else {
        this.distance_f = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Distances
    // Serialize message field [distance_a]
    bufferOffset = _serializer.int32(obj.distance_a, buffer, bufferOffset);
    // Serialize message field [distance_b]
    bufferOffset = _serializer.int32(obj.distance_b, buffer, bufferOffset);
    // Serialize message field [distance_c]
    bufferOffset = _serializer.int32(obj.distance_c, buffer, bufferOffset);
    // Serialize message field [distance_d]
    bufferOffset = _serializer.int32(obj.distance_d, buffer, bufferOffset);
    // Serialize message field [distance_e]
    bufferOffset = _serializer.int32(obj.distance_e, buffer, bufferOffset);
    // Serialize message field [distance_f]
    bufferOffset = _serializer.int32(obj.distance_f, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Distances
    let len;
    let data = new Distances(null);
    // Deserialize message field [distance_a]
    data.distance_a = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [distance_b]
    data.distance_b = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [distance_c]
    data.distance_c = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [distance_d]
    data.distance_d = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [distance_e]
    data.distance_e = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [distance_f]
    data.distance_f = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'serial_comms/Distances';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '88930f00743784851f3f6d92c51802bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Defines a message for holding the six distance values.
    int32 distance_a
    int32 distance_b
    int32 distance_c
    int32 distance_d
    int32 distance_e
    int32 distance_f
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Distances(null);
    if (msg.distance_a !== undefined) {
      resolved.distance_a = msg.distance_a;
    }
    else {
      resolved.distance_a = 0
    }

    if (msg.distance_b !== undefined) {
      resolved.distance_b = msg.distance_b;
    }
    else {
      resolved.distance_b = 0
    }

    if (msg.distance_c !== undefined) {
      resolved.distance_c = msg.distance_c;
    }
    else {
      resolved.distance_c = 0
    }

    if (msg.distance_d !== undefined) {
      resolved.distance_d = msg.distance_d;
    }
    else {
      resolved.distance_d = 0
    }

    if (msg.distance_e !== undefined) {
      resolved.distance_e = msg.distance_e;
    }
    else {
      resolved.distance_e = 0
    }

    if (msg.distance_f !== undefined) {
      resolved.distance_f = msg.distance_f;
    }
    else {
      resolved.distance_f = 0
    }

    return resolved;
    }
};

module.exports = Distances;
