// Generated by gencpp from file iiwa_tools/GetFKRequest.msg
// DO NOT EDIT!


#ifndef IIWA_TOOLS_MESSAGE_GETFKREQUEST_H
#define IIWA_TOOLS_MESSAGE_GETFKREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Float64MultiArray.h>

namespace iiwa_tools
{
template <class ContainerAllocator>
struct GetFKRequest_
{
  typedef GetFKRequest_<ContainerAllocator> Type;

  GetFKRequest_()
    : joints()  {
    }
  GetFKRequest_(const ContainerAllocator& _alloc)
    : joints(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Float64MultiArray_<ContainerAllocator>  _joints_type;
  _joints_type joints;





  typedef boost::shared_ptr< ::iiwa_tools::GetFKRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::iiwa_tools::GetFKRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetFKRequest_

typedef ::iiwa_tools::GetFKRequest_<std::allocator<void> > GetFKRequest;

typedef boost::shared_ptr< ::iiwa_tools::GetFKRequest > GetFKRequestPtr;
typedef boost::shared_ptr< ::iiwa_tools::GetFKRequest const> GetFKRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::iiwa_tools::GetFKRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::iiwa_tools::GetFKRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::iiwa_tools::GetFKRequest_<ContainerAllocator1> & lhs, const ::iiwa_tools::GetFKRequest_<ContainerAllocator2> & rhs)
{
  return lhs.joints == rhs.joints;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::iiwa_tools::GetFKRequest_<ContainerAllocator1> & lhs, const ::iiwa_tools::GetFKRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace iiwa_tools

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::iiwa_tools::GetFKRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::iiwa_tools::GetFKRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::iiwa_tools::GetFKRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::iiwa_tools::GetFKRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::iiwa_tools::GetFKRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::iiwa_tools::GetFKRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::iiwa_tools::GetFKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c3ed12c296ea0cdf8cae379653742d3d";
  }

  static const char* value(const ::iiwa_tools::GetFKRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc3ed12c296ea0cdfULL;
  static const uint64_t static_value2 = 0x8cae379653742d3dULL;
};

template<class ContainerAllocator>
struct DataType< ::iiwa_tools::GetFKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "iiwa_tools/GetFKRequest";
  }

  static const char* value(const ::iiwa_tools::GetFKRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::iiwa_tools::GetFKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Float64MultiArray joints\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float64MultiArray\n"
"# Please look at the MultiArrayLayout message definition for\n"
"# documentation on all multiarrays.\n"
"\n"
"MultiArrayLayout  layout        # specification of data layout\n"
"float64[]         data          # array of data\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayLayout\n"
"# The multiarray declares a generic multi-dimensional array of a\n"
"# particular data type.  Dimensions are ordered from outer most\n"
"# to inner most.\n"
"\n"
"MultiArrayDimension[] dim # Array of dimension properties\n"
"uint32 data_offset        # padding elements at front of data\n"
"\n"
"# Accessors should ALWAYS be written in terms of dimension stride\n"
"# and specified outer-most dimension first.\n"
"# \n"
"# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n"
"#\n"
"# A standard, 3-channel 640x480 image with interleaved color channels\n"
"# would be specified as:\n"
"#\n"
"# dim[0].label  = \"height\"\n"
"# dim[0].size   = 480\n"
"# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n"
"# dim[1].label  = \"width\"\n"
"# dim[1].size   = 640\n"
"# dim[1].stride = 3*640 = 1920\n"
"# dim[2].label  = \"channel\"\n"
"# dim[2].size   = 3\n"
"# dim[2].stride = 3\n"
"#\n"
"# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayDimension\n"
"string label   # label of given dimension\n"
"uint32 size    # size of given dimension (in type units)\n"
"uint32 stride  # stride of given dimension\n"
;
  }

  static const char* value(const ::iiwa_tools::GetFKRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::iiwa_tools::GetFKRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joints);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetFKRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::iiwa_tools::GetFKRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::iiwa_tools::GetFKRequest_<ContainerAllocator>& v)
  {
    s << indent << "joints: ";
    s << std::endl;
    Printer< ::std_msgs::Float64MultiArray_<ContainerAllocator> >::stream(s, indent + "  ", v.joints);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IIWA_TOOLS_MESSAGE_GETFKREQUEST_H
