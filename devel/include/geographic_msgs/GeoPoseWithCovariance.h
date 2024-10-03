// Generated by gencpp from file geographic_msgs/GeoPoseWithCovariance.msg
// DO NOT EDIT!


#ifndef GEOGRAPHIC_MSGS_MESSAGE_GEOPOSEWITHCOVARIANCE_H
#define GEOGRAPHIC_MSGS_MESSAGE_GEOPOSEWITHCOVARIANCE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geographic_msgs/GeoPose.h>

namespace geographic_msgs
{
template <class ContainerAllocator>
struct GeoPoseWithCovariance_
{
  typedef GeoPoseWithCovariance_<ContainerAllocator> Type;

  GeoPoseWithCovariance_()
    : pose()
    , covariance()  {
      covariance.assign(0.0);
  }
  GeoPoseWithCovariance_(const ContainerAllocator& _alloc)
    : pose(_alloc)
    , covariance()  {
  (void)_alloc;
      covariance.assign(0.0);
  }



   typedef  ::geographic_msgs::GeoPose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef boost::array<double, 36>  _covariance_type;
  _covariance_type covariance;





  typedef boost::shared_ptr< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> const> ConstPtr;

}; // struct GeoPoseWithCovariance_

typedef ::geographic_msgs::GeoPoseWithCovariance_<std::allocator<void> > GeoPoseWithCovariance;

typedef boost::shared_ptr< ::geographic_msgs::GeoPoseWithCovariance > GeoPoseWithCovariancePtr;
typedef boost::shared_ptr< ::geographic_msgs::GeoPoseWithCovariance const> GeoPoseWithCovarianceConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator1> & lhs, const ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator2> & rhs)
{
  return lhs.pose == rhs.pose &&
    lhs.covariance == rhs.covariance;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator1> & lhs, const ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace geographic_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "84c133e7220362d3d8aff8e415075357";
  }

  static const char* value(const ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x84c133e7220362d3ULL;
  static const uint64_t static_value2 = 0xd8aff8e415075357ULL;
};

template<class ContainerAllocator>
struct DataType< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geographic_msgs/GeoPoseWithCovariance";
  }

  static const char* value(const ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Geographic pose, using the WGS 84 reference ellipsoid.\n"
"#\n"
"# Orientation uses the East-North-Up (ENU) frame of reference.\n"
"# (But, what about singularities at the poles?)\n"
"\n"
"GeoPose pose\n"
"\n"
"# Row-major representation of the 6x6 covariance matrix\n"
"# The orientation parameters use a fixed-axis representation.\n"
"# In order, the parameters are:\n"
"# (Lat, Lon, Alt, rotation about E (East) axis, rotation about N (North) axis, rotation about U (Up) axis)\n"
"float64[36] covariance\n"
"\n"
"================================================================================\n"
"MSG: geographic_msgs/GeoPose\n"
"# Geographic pose, using the WGS 84 reference ellipsoid.\n"
"#\n"
"# Orientation uses the East-North-Up (ENU) frame of reference.\n"
"# (But, what about singularities at the poles?)\n"
"\n"
"GeoPoint position\n"
"geometry_msgs/Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geographic_msgs/GeoPoint\n"
"# Geographic point, using the WGS 84 reference ellipsoid.\n"
"\n"
"# Latitude [degrees]. Positive is north of equator; negative is south\n"
"# (-90 <= latitude <= +90).\n"
"float64 latitude\n"
"\n"
"# Longitude [degrees]. Positive is east of prime meridian; negative is\n"
"# west (-180 <= longitude <= +180). At the poles, latitude is -90 or\n"
"# +90, and longitude is irrelevant, but must be in range.\n"
"float64 longitude\n"
"\n"
"# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).\n"
"float64 altitude\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
      stream.next(m.covariance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GeoPoseWithCovariance_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::geographic_msgs::GeoPoseWithCovariance_<ContainerAllocator>& v)
  {
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geographic_msgs::GeoPose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "covariance[]" << std::endl;
    for (size_t i = 0; i < v.covariance.size(); ++i)
    {
      s << indent << "  covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.covariance[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // GEOGRAPHIC_MSGS_MESSAGE_GEOPOSEWITHCOVARIANCE_H
