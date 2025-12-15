// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from explorer_interfaces:action/Discover.idl
// generated code does not contain a copyright notice

#ifndef EXPLORER_INTERFACES__ACTION__DETAIL__DISCOVER__STRUCT_HPP_
#define EXPLORER_INTERFACES__ACTION__DETAIL__DISCOVER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__explorer_interfaces__action__Discover_Goal __attribute__((deprecated))
#else
# define DEPRECATED__explorer_interfaces__action__Discover_Goal __declspec(deprecated)
#endif

namespace explorer_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Discover_Goal_
{
  using Type = Discover_Goal_<ContainerAllocator>;

  explicit Discover_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->strategy = 0;
      this->map_completed_thres = 0.0f;
    }
  }

  explicit Discover_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->strategy = 0;
      this->map_completed_thres = 0.0f;
    }
  }

  // field types and members
  using _strategy_type =
    uint8_t;
  _strategy_type strategy;
  using _map_completed_thres_type =
    float;
  _map_completed_thres_type map_completed_thres;

  // setters for named parameter idiom
  Type & set__strategy(
    const uint8_t & _arg)
  {
    this->strategy = _arg;
    return *this;
  }
  Type & set__map_completed_thres(
    const float & _arg)
  {
    this->map_completed_thres = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    explorer_interfaces::action::Discover_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const explorer_interfaces::action::Discover_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__explorer_interfaces__action__Discover_Goal
    std::shared_ptr<explorer_interfaces::action::Discover_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__explorer_interfaces__action__Discover_Goal
    std::shared_ptr<explorer_interfaces::action::Discover_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Discover_Goal_ & other) const
  {
    if (this->strategy != other.strategy) {
      return false;
    }
    if (this->map_completed_thres != other.map_completed_thres) {
      return false;
    }
    return true;
  }
  bool operator!=(const Discover_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Discover_Goal_

// alias to use template instance with default allocator
using Discover_Goal =
  explorer_interfaces::action::Discover_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace explorer_interfaces


#ifndef _WIN32
# define DEPRECATED__explorer_interfaces__action__Discover_Result __attribute__((deprecated))
#else
# define DEPRECATED__explorer_interfaces__action__Discover_Result __declspec(deprecated)
#endif

namespace explorer_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Discover_Result_
{
  using Type = Discover_Result_<ContainerAllocator>;

  explicit Discover_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  explicit Discover_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  // field types and members
  using _result_type =
    bool;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const bool & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    explorer_interfaces::action::Discover_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const explorer_interfaces::action::Discover_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__explorer_interfaces__action__Discover_Result
    std::shared_ptr<explorer_interfaces::action::Discover_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__explorer_interfaces__action__Discover_Result
    std::shared_ptr<explorer_interfaces::action::Discover_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Discover_Result_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Discover_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Discover_Result_

// alias to use template instance with default allocator
using Discover_Result =
  explorer_interfaces::action::Discover_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace explorer_interfaces


#ifndef _WIN32
# define DEPRECATED__explorer_interfaces__action__Discover_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__explorer_interfaces__action__Discover_Feedback __declspec(deprecated)
#endif

namespace explorer_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Discover_Feedback_
{
  using Type = Discover_Feedback_<ContainerAllocator>;

  explicit Discover_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->progress = 0.0f;
    }
  }

  explicit Discover_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->progress = 0.0f;
    }
  }

  // field types and members
  using _progress_type =
    float;
  _progress_type progress;

  // setters for named parameter idiom
  Type & set__progress(
    const float & _arg)
  {
    this->progress = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    explorer_interfaces::action::Discover_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const explorer_interfaces::action::Discover_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__explorer_interfaces__action__Discover_Feedback
    std::shared_ptr<explorer_interfaces::action::Discover_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__explorer_interfaces__action__Discover_Feedback
    std::shared_ptr<explorer_interfaces::action::Discover_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Discover_Feedback_ & other) const
  {
    if (this->progress != other.progress) {
      return false;
    }
    return true;
  }
  bool operator!=(const Discover_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Discover_Feedback_

// alias to use template instance with default allocator
using Discover_Feedback =
  explorer_interfaces::action::Discover_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace explorer_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "explorer_interfaces/action/detail/discover__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__explorer_interfaces__action__Discover_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__explorer_interfaces__action__Discover_SendGoal_Request __declspec(deprecated)
#endif

namespace explorer_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Discover_SendGoal_Request_
{
  using Type = Discover_SendGoal_Request_<ContainerAllocator>;

  explicit Discover_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit Discover_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    explorer_interfaces::action::Discover_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const explorer_interfaces::action::Discover_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__explorer_interfaces__action__Discover_SendGoal_Request
    std::shared_ptr<explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__explorer_interfaces__action__Discover_SendGoal_Request
    std::shared_ptr<explorer_interfaces::action::Discover_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Discover_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Discover_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Discover_SendGoal_Request_

// alias to use template instance with default allocator
using Discover_SendGoal_Request =
  explorer_interfaces::action::Discover_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace explorer_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__explorer_interfaces__action__Discover_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__explorer_interfaces__action__Discover_SendGoal_Response __declspec(deprecated)
#endif

namespace explorer_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Discover_SendGoal_Response_
{
  using Type = Discover_SendGoal_Response_<ContainerAllocator>;

  explicit Discover_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit Discover_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__explorer_interfaces__action__Discover_SendGoal_Response
    std::shared_ptr<explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__explorer_interfaces__action__Discover_SendGoal_Response
    std::shared_ptr<explorer_interfaces::action::Discover_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Discover_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Discover_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Discover_SendGoal_Response_

// alias to use template instance with default allocator
using Discover_SendGoal_Response =
  explorer_interfaces::action::Discover_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace explorer_interfaces

namespace explorer_interfaces
{

namespace action
{

struct Discover_SendGoal
{
  using Request = explorer_interfaces::action::Discover_SendGoal_Request;
  using Response = explorer_interfaces::action::Discover_SendGoal_Response;
};

}  // namespace action

}  // namespace explorer_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__explorer_interfaces__action__Discover_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__explorer_interfaces__action__Discover_GetResult_Request __declspec(deprecated)
#endif

namespace explorer_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Discover_GetResult_Request_
{
  using Type = Discover_GetResult_Request_<ContainerAllocator>;

  explicit Discover_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit Discover_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__explorer_interfaces__action__Discover_GetResult_Request
    std::shared_ptr<explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__explorer_interfaces__action__Discover_GetResult_Request
    std::shared_ptr<explorer_interfaces::action::Discover_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Discover_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Discover_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Discover_GetResult_Request_

// alias to use template instance with default allocator
using Discover_GetResult_Request =
  explorer_interfaces::action::Discover_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace explorer_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "explorer_interfaces/action/detail/discover__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__explorer_interfaces__action__Discover_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__explorer_interfaces__action__Discover_GetResult_Response __declspec(deprecated)
#endif

namespace explorer_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Discover_GetResult_Response_
{
  using Type = Discover_GetResult_Response_<ContainerAllocator>;

  explicit Discover_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit Discover_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    explorer_interfaces::action::Discover_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const explorer_interfaces::action::Discover_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__explorer_interfaces__action__Discover_GetResult_Response
    std::shared_ptr<explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__explorer_interfaces__action__Discover_GetResult_Response
    std::shared_ptr<explorer_interfaces::action::Discover_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Discover_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Discover_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Discover_GetResult_Response_

// alias to use template instance with default allocator
using Discover_GetResult_Response =
  explorer_interfaces::action::Discover_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace explorer_interfaces

namespace explorer_interfaces
{

namespace action
{

struct Discover_GetResult
{
  using Request = explorer_interfaces::action::Discover_GetResult_Request;
  using Response = explorer_interfaces::action::Discover_GetResult_Response;
};

}  // namespace action

}  // namespace explorer_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "explorer_interfaces/action/detail/discover__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__explorer_interfaces__action__Discover_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__explorer_interfaces__action__Discover_FeedbackMessage __declspec(deprecated)
#endif

namespace explorer_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Discover_FeedbackMessage_
{
  using Type = Discover_FeedbackMessage_<ContainerAllocator>;

  explicit Discover_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit Discover_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    explorer_interfaces::action::Discover_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const explorer_interfaces::action::Discover_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__explorer_interfaces__action__Discover_FeedbackMessage
    std::shared_ptr<explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__explorer_interfaces__action__Discover_FeedbackMessage
    std::shared_ptr<explorer_interfaces::action::Discover_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Discover_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Discover_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Discover_FeedbackMessage_

// alias to use template instance with default allocator
using Discover_FeedbackMessage =
  explorer_interfaces::action::Discover_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace explorer_interfaces

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace explorer_interfaces
{

namespace action
{

struct Discover
{
  /// The goal message defined in the action definition.
  using Goal = explorer_interfaces::action::Discover_Goal;
  /// The result message defined in the action definition.
  using Result = explorer_interfaces::action::Discover_Result;
  /// The feedback message defined in the action definition.
  using Feedback = explorer_interfaces::action::Discover_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = explorer_interfaces::action::Discover_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = explorer_interfaces::action::Discover_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = explorer_interfaces::action::Discover_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct Discover Discover;

}  // namespace action

}  // namespace explorer_interfaces

#endif  // EXPLORER_INTERFACES__ACTION__DETAIL__DISCOVER__STRUCT_HPP_
