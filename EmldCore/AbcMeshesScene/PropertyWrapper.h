//-*****************************************************************************
// Copyright (c) 2001-2013, Christopher Jon Horvath. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of Christopher Jon Horvath nor the names of his
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//-*****************************************************************************

#ifndef _EmldCore_AbcMeshesScene_PropertyWrapper_h_
#define _EmldCore_AbcMeshesScene_PropertyWrapper_h_

#include "Foundation.h"
#include "Interpolation.h"

namespace EmldCore {
namespace AbcMeshesScene {

namespace {

template <typename FROM, typename TO>
inline TO value_cast(const FROM& i_from) {
    return static_cast<TO>(i_from);
}

//-*****************************************************************************
// BOOL VALUE CASTS
//-*****************************************************************************

template <>
inline V3f value_cast<bool_t, V3f>(const bool_t& i_v) {
    return V3f(static_cast<float>(i_v));
}

template <>
inline std::string value_cast<bool_t, std::string>(const bool_t& i_v) {
    return i_v ? std::string("true") : std::string("false");
}

//-*****************************************************************************
// INT VALUE CASTS
//-*****************************************************************************

template <>
inline V3f value_cast<int32_t, V3f>(const int32_t& i_v) {
    return V3f(static_cast<float>(i_v));
}

template <>
inline std::string value_cast<int32_t, std::string>(const int32_t& i_v) {
    return (boost::format("%d") % i_v).str();
}

//-*****************************************************************************
// INT64 VALUE CASTS
//-*****************************************************************************

template <>
inline V3f value_cast<int64_t, V3f>(const int64_t& i_v) {
    return V3f(static_cast<float>(i_v));
}

template <>
inline std::string value_cast<int64_t, std::string>(const int64_t& i_v) {
    return (boost::format("%d") % i_v).str();
}

//-*****************************************************************************
// FLOAT VALUE CASTS
//-*****************************************************************************

template <>
inline V3f value_cast<float, V3f>(const float& i_v) {
    return V3f(i_v);
}

template <>
inline std::string value_cast<float, std::string>(const float& i_v) {
    return (boost::format("%f") % i_v).str();
}

//-*****************************************************************************
// DOUBLE VALUE CASTS
//-*****************************************************************************

template <>
inline V3f value_cast<double, V3f>(const double& i_v) {
    return V3f(static_cast<float>(i_v));
}

template <>
inline std::string value_cast<double, std::string>(const double& i_v) {
    return (boost::format("%f") % i_v).str();
}

//-*****************************************************************************
// V3F VALUE CASTS
//-*****************************************************************************

template <>
inline bool value_cast<V3f, bool>(const V3f&) {
    return false;
}

template <>
inline int value_cast<V3f, int>(const V3f&) {
    return 0;
}

template <>
inline float value_cast<V3f, float>(const V3f&) {
    return 0.0f;
}

template <>
inline std::string value_cast<V3f, std::string>(const V3f& i_v) {
    return (boost::format("(%f,%f,%f)") % i_v.x % i_v.y % i_v.z).str();
}

//-*****************************************************************************
// V3D VALUE CASTS
//-*****************************************************************************

template <>
inline bool value_cast<V3d, bool>(const V3d&) {
    return false;
}

template <>
inline int value_cast<V3d, int>(const V3d&) {
    return 0;
}

template <>
inline float value_cast<V3d, float>(const V3d&) {
    return 0.0f;
}

template <>
inline V3f value_cast<V3d, V3f>(const V3d& i_v) {
    return V3f(i_v);
}

template <>
inline std::string value_cast<V3d, std::string>(const V3d& i_v) {
    return (boost::format("(%f,%f,%f)") % i_v.x % i_v.y % i_v.z).str();
}

//-*****************************************************************************
// C3F VALUE CASTS
//-*****************************************************************************

template <>
inline bool value_cast<C3f, bool>(const C3f&) {
    return false;
}

template <>
inline int value_cast<C3f, int>(const C3f&) {
    return 0;
}

template <>
inline float value_cast<C3f, float>(const C3f&) {
    return 0.0f;
}

template <>
inline V3f value_cast<C3f, V3f>(const C3f& i_v) {
    return C3f(i_v);
}

template <>
inline std::string value_cast<C3f, std::string>(const C3f& i_v) {
    return (boost::format("(%f,%f,%f)") % i_v.x % i_v.y % i_v.z).str();
}

//-*****************************************************************************
// STRING VALUE CASTS
//-*****************************************************************************

template <>
inline bool value_cast<std::string, bool>(const std::string& i_v) {
    return (i_v != "");
}

template <>
inline int value_cast<std::string, int>(const std::string& i_v) {
    return atoi(i_v.c_str());
}

template <>
inline float value_cast<std::string, float>(const std::string& i_v) {
    return static_cast<float>(atof(i_v.c_str()));
}

template <>
inline V3f value_cast<std::string, V3f>(const std::string& i_v) {
    boost::char_delimiters_separator<char> sep("(, )");
    boost::tokenizer<> tokens(i_v, sep);
    V3f ret(0.0f);
    int i = 0;
    for (boost::tokenizer<>::const_iterator it = tokens.begin();
         i < 3 && it != tokens.end();
         ++it, ++i) {
        ret[i] = static_cast<float>(atof((*it).c_str()));
    }
    return ret;
}

template <>
inline std::string value_cast<std::string, std::string>(
  const std::string& i_v) {
    return i_v;
}

}  // End anonymous namespace

//-*****************************************************************************
// BASE PROPERTY WRAPPER
//-*****************************************************************************

class BasePropertyWrapper {
public:
    BasePropertyWrapper() {
    }
    virtual ~BasePropertyWrapper() {
    }

    virtual void setTime(chrono_t i_time) = 0;
    virtual bool getValueBool() const = 0;
    virtual int getValueInt() const = 0;
    virtual float getValueFloat() const = 0;
    virtual V3f getValueVec() const = 0;
    virtual std::string getValueString() const = 0;
};

//-*****************************************************************************
typedef ABCM_SHARED_PTR<BasePropertyWrapper> BasePropertyWrapperSptr;

//-*****************************************************************************
// TEMPLATED AGAINST ALEMBIC SCALAR PROPERTY TYPE
//-*****************************************************************************

template <typename VPROP>
class TypedPropertyWrapper : public BasePropertyWrapper {
public:
    typedef typename VPROP::value_type internal_value_type;

    TypedPropertyWrapper(VPROP& i_vprop, chrono_t i_time)
      : BasePropertyWrapper()
      , m_abcProperty(i_vprop)
      , m_currentTime(i_time) {
        if (m_abcProperty.isConstant()) {
            m_value = m_abcProperty.getValue();
        } else {
            m_value = Interpolate(m_abcProperty, m_currentTime);
        }
    }

    void setTime(chrono_t i_time) {
        if (i_time == m_currentTime) { return; }

        if (!m_abcProperty.isConstant()) {
            m_value = Interpolate(m_abcProperty, i_time);
        }
        m_currentTime = i_time;
    }

    bool getValueBool() const {
        return value_cast<internal_value_type, bool>(m_value);
    }

    int getValueInt() const {
        return value_cast<internal_value_type, int>(m_value);
    }

    float getValueFloat() const {
        return value_cast<internal_value_type, float>(m_value);
    }

    V3f getValueVec() const {
        return value_cast<internal_value_type, V3f>(m_value);
    }

    std::string getValueString() const {
        return value_cast<internal_value_type, std::string>(m_value);
    }

protected:
    VPROP m_abcProperty;
    internal_value_type m_value;
    chrono_t m_currentTime;
};

//-*****************************************************************************
// TEMPLATED AGAINST ALEMBIC SCALAR PROPERTY TYPE
//-*****************************************************************************

// CJH HUGE HACK
class V3fArrayPropertyWrapper : public BasePropertyWrapper {
public:
    typedef V3f internal_value_type;

    V3fArrayPropertyWrapper(AbcG::IV3fArrayProperty& i_vprop, chrono_t i_time)
      : BasePropertyWrapper()
      , m_abcProperty(i_vprop)
      , m_currentTime(i_time) {
        // if ( m_abcProperty.isConstant() )
        {
            AbcG::V3fArraySamplePtr samp = m_abcProperty.getValue();
            if (samp) { m_value = (*samp)[0]; }
        }
        // else
        //{
        //    m_value = Interpolate( m_abcProperty, m_currentTime );
        //}
    }

    void setTime(chrono_t i_time) {
        // if ( i_time == m_currentTime ) { return; }
        //
        // if ( !m_abcProperty.isConstant() )
        //{
        //    m_value = Interpolate( m_abcProperty, i_time );
        //}
        m_currentTime = i_time;
    }

    bool getValueBool() const {
        return value_cast<internal_value_type, bool>(m_value);
    }

    int getValueInt() const {
        return value_cast<internal_value_type, int>(m_value);
    }

    float getValueFloat() const {
        return value_cast<internal_value_type, float>(m_value);
    }

    V3f getValueVec() const {
        return value_cast<internal_value_type, V3f>(m_value);
    }

    std::string getValueString() const {
        return value_cast<internal_value_type, std::string>(m_value);
    }

protected:
    AbcG::IV3fArrayProperty m_abcProperty;
    V3f m_value;
    chrono_t m_currentTime;
};

//-*****************************************************************************
typedef TypedPropertyWrapper<AbcG::IBoolProperty> BoolPropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IInt32Property> Int32PropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IInt64Property> Int64PropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IFloatProperty> FloatPropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IDoubleProperty> DoublePropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IV3fProperty> V3fPropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IV3dProperty> V3dPropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IP3fProperty> P3fPropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IP3dProperty> P3dPropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IN3fProperty> N3fPropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IN3dProperty> N3dPropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IC3fProperty> C3fPropertyWrapper;
typedef TypedPropertyWrapper<AbcG::IStringProperty> StringPropertyWrapper;

}  // End namespace AbcMeshesScene
}  // End namespace EmldCore

#endif
