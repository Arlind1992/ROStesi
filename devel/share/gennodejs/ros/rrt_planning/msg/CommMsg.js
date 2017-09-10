// Auto-generated. Do not edit!

// (in-package rrt_planning.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class CommMsg {
  constructor() {
    this.cellX = 0;
    this.cellY = 0;
    this.uploadTime = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type CommMsg
    // Serialize message field [cellX]
    bufferInfo = _serializer.uint32(obj.cellX, bufferInfo);
    // Serialize message field [cellY]
    bufferInfo = _serializer.uint32(obj.cellY, bufferInfo);
    // Serialize message field [uploadTime]
    bufferInfo = _serializer.uint32(obj.uploadTime, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type CommMsg
    let tmp;
    let len;
    let data = new CommMsg();
    // Deserialize message field [cellX]
    tmp = _deserializer.uint32(buffer);
    data.cellX = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [cellY]
    tmp = _deserializer.uint32(buffer);
    data.cellY = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [uploadTime]
    tmp = _deserializer.uint32(buffer);
    data.uploadTime = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'rrt_planning/CommMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ae8e81eb815f3fe2d8493f75b85e1c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32  cellX	
    uint32  cellY
    uint32 uploadTime
    
    `;
  }

};

module.exports = CommMsg;
