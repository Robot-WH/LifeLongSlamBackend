// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: sc.proto

#ifndef PROTOBUF_sc_2eproto__INCLUDED
#define PROTOBUF_sc_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3001000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3001000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace lifelong_backend {
namespace scan_context {
namespace proto {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_sc_2eproto();
void protobuf_InitDefaults_sc_2eproto();
void protobuf_AssignDesc_sc_2eproto();
void protobuf_ShutdownFile_sc_2eproto();

class scan_context;

// ===================================================================

class scan_context : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:lifelong_backend.scan_context.proto.scan_context) */ {
 public:
  scan_context();
  virtual ~scan_context();

  scan_context(const scan_context& from);

  inline scan_context& operator=(const scan_context& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const scan_context& default_instance();

  static const scan_context* internal_default_instance();

  void Swap(scan_context* other);

  // implements Message ----------------------------------------------

  inline scan_context* New() const { return New(NULL); }

  scan_context* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const scan_context& from);
  void MergeFrom(const scan_context& from);
  void Clear();
  bool IsInitialized() const;

  size_t ByteSizeLong() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(scan_context* other);
  void UnsafeMergeFrom(const scan_context& from);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated double polarcontext = 1;
  int polarcontext_size() const;
  void clear_polarcontext();
  static const int kPolarcontextFieldNumber = 1;
  double polarcontext(int index) const;
  void set_polarcontext(int index, double value);
  void add_polarcontext(double value);
  const ::google::protobuf::RepeatedField< double >&
      polarcontext() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_polarcontext();

  // repeated double ringkey = 2;
  int ringkey_size() const;
  void clear_ringkey();
  static const int kRingkeyFieldNumber = 2;
  double ringkey(int index) const;
  void set_ringkey(int index, double value);
  void add_ringkey(double value);
  const ::google::protobuf::RepeatedField< double >&
      ringkey() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_ringkey();

  // @@protoc_insertion_point(class_scope:lifelong_backend.scan_context.proto.scan_context)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedField< double > polarcontext_;
  mutable int _polarcontext_cached_byte_size_;
  ::google::protobuf::RepeatedField< double > ringkey_;
  mutable int _ringkey_cached_byte_size_;
  mutable int _cached_size_;
  friend void  protobuf_InitDefaults_sc_2eproto_impl();
  friend void  protobuf_AddDesc_sc_2eproto_impl();
  friend void protobuf_AssignDesc_sc_2eproto();
  friend void protobuf_ShutdownFile_sc_2eproto();

  void InitAsDefaultInstance();
};
extern ::google::protobuf::internal::ExplicitlyConstructed<scan_context> scan_context_default_instance_;

// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// scan_context

// repeated double polarcontext = 1;
inline int scan_context::polarcontext_size() const {
  return polarcontext_.size();
}
inline void scan_context::clear_polarcontext() {
  polarcontext_.Clear();
}
inline double scan_context::polarcontext(int index) const {
  // @@protoc_insertion_point(field_get:lifelong_backend.scan_context.proto.scan_context.polarcontext)
  return polarcontext_.Get(index);
}
inline void scan_context::set_polarcontext(int index, double value) {
  polarcontext_.Set(index, value);
  // @@protoc_insertion_point(field_set:lifelong_backend.scan_context.proto.scan_context.polarcontext)
}
inline void scan_context::add_polarcontext(double value) {
  polarcontext_.Add(value);
  // @@protoc_insertion_point(field_add:lifelong_backend.scan_context.proto.scan_context.polarcontext)
}
inline const ::google::protobuf::RepeatedField< double >&
scan_context::polarcontext() const {
  // @@protoc_insertion_point(field_list:lifelong_backend.scan_context.proto.scan_context.polarcontext)
  return polarcontext_;
}
inline ::google::protobuf::RepeatedField< double >*
scan_context::mutable_polarcontext() {
  // @@protoc_insertion_point(field_mutable_list:lifelong_backend.scan_context.proto.scan_context.polarcontext)
  return &polarcontext_;
}

// repeated double ringkey = 2;
inline int scan_context::ringkey_size() const {
  return ringkey_.size();
}
inline void scan_context::clear_ringkey() {
  ringkey_.Clear();
}
inline double scan_context::ringkey(int index) const {
  // @@protoc_insertion_point(field_get:lifelong_backend.scan_context.proto.scan_context.ringkey)
  return ringkey_.Get(index);
}
inline void scan_context::set_ringkey(int index, double value) {
  ringkey_.Set(index, value);
  // @@protoc_insertion_point(field_set:lifelong_backend.scan_context.proto.scan_context.ringkey)
}
inline void scan_context::add_ringkey(double value) {
  ringkey_.Add(value);
  // @@protoc_insertion_point(field_add:lifelong_backend.scan_context.proto.scan_context.ringkey)
}
inline const ::google::protobuf::RepeatedField< double >&
scan_context::ringkey() const {
  // @@protoc_insertion_point(field_list:lifelong_backend.scan_context.proto.scan_context.ringkey)
  return ringkey_;
}
inline ::google::protobuf::RepeatedField< double >*
scan_context::mutable_ringkey() {
  // @@protoc_insertion_point(field_mutable_list:lifelong_backend.scan_context.proto.scan_context.ringkey)
  return &ringkey_;
}

inline const scan_context* scan_context::internal_default_instance() {
  return &scan_context_default_instance_.get();
}
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace scan_context
}  // namespace lifelong_backend

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_sc_2eproto__INCLUDED
