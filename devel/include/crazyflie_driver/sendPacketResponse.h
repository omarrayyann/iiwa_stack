// Generated by gencpp from file crazyflie_driver/sendPacketResponse.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_SENDPACKETRESPONSE_H
#define CRAZYFLIE_DRIVER_MESSAGE_SENDPACKETRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace crazyflie_driver
{
template <class ContainerAllocator>
struct sendPacketResponse_
{
  typedef sendPacketResponse_<ContainerAllocator> Type;

  sendPacketResponse_()
    {
    }
  sendPacketResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> const> ConstPtr;

}; // struct sendPacketResponse_

typedef ::crazyflie_driver::sendPacketResponse_<std::allocator<void> > sendPacketResponse;

typedef boost::shared_ptr< ::crazyflie_driver::sendPacketResponse > sendPacketResponsePtr;
typedef boost::shared_ptr< ::crazyflie_driver::sendPacketResponse const> sendPacketResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace crazyflie_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::crazyflie_driver::sendPacketResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_driver/sendPacketResponse";
  }

  static const char* value(const ::crazyflie_driver::sendPacketResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::crazyflie_driver::sendPacketResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct sendPacketResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyflie_driver::sendPacketResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::crazyflie_driver::sendPacketResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_SENDPACKETRESPONSE_H
