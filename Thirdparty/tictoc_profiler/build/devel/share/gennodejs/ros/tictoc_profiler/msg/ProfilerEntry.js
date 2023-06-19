// Auto-generated. Do not edit!

// (in-package tictoc_profiler.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ProfilerEntry {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.seq = null;
      this.name = null;
      this.start_time = null;
      this.end_time = null;
      this.delta_time_ms = null;
    }
    else {
      if (initObj.hasOwnProperty('seq')) {
        this.seq = initObj.seq
      }
      else {
        this.seq = 0;
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = 0;
      }
      if (initObj.hasOwnProperty('end_time')) {
        this.end_time = initObj.end_time
      }
      else {
        this.end_time = 0;
      }
      if (initObj.hasOwnProperty('delta_time_ms')) {
        this.delta_time_ms = initObj.delta_time_ms
      }
      else {
        this.delta_time_ms = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ProfilerEntry
    // Serialize message field [seq]
    bufferOffset = _serializer.uint32(obj.seq, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [start_time]
    bufferOffset = _serializer.int64(obj.start_time, buffer, bufferOffset);
    // Serialize message field [end_time]
    bufferOffset = _serializer.int64(obj.end_time, buffer, bufferOffset);
    // Serialize message field [delta_time_ms]
    bufferOffset = _serializer.float64(obj.delta_time_ms, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ProfilerEntry
    let len;
    let data = new ProfilerEntry(null);
    // Deserialize message field [seq]
    data.seq = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [end_time]
    data.end_time = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [delta_time_ms]
    data.delta_time_ms = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tictoc_profiler/ProfilerEntry';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bfc3f9f9968c9db2e3db18a9276d6e48';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 seq
    string name
    int64 start_time
    int64 end_time
    float64 delta_time_ms
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ProfilerEntry(null);
    if (msg.seq !== undefined) {
      resolved.seq = msg.seq;
    }
    else {
      resolved.seq = 0
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = 0
    }

    if (msg.end_time !== undefined) {
      resolved.end_time = msg.end_time;
    }
    else {
      resolved.end_time = 0
    }

    if (msg.delta_time_ms !== undefined) {
      resolved.delta_time_ms = msg.delta_time_ms;
    }
    else {
      resolved.delta_time_ms = 0.0
    }

    return resolved;
    }
};

module.exports = ProfilerEntry;
