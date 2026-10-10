/*
* Copyright (c) 2024, Autonomous Robots Lab, Norwegian University of Science and Technology
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ROVIO_BAROUPDATE_HPP_
#define ROVIO_BAROUPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "rovio/FilterStates.hpp"

namespace rovio {

/** \brief Class, defining the innovation.
 */
class BaroInnovation: public LWF::State<LWF::VectorElement<3>>{
 public:
  typedef LWF::State<LWF::VectorElement<3>> Base;
  using Base::E_;
  static constexpr unsigned int _pos = 0;
  BaroInnovation(){
    static_assert(_pos+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
  };
  virtual ~BaroInnovation(){};
  inline V3D& pos(){
    return this->template get<_pos>();
  }
  inline const V3D& pos() const{
    return this->template get<_pos>();
  }
};

/** \brief Class, dummy auxillary class for Zero Baro update
 */
class BaroUpdateMeasAuxiliary: public LWF::AuxiliaryBase<BaroUpdateMeasAuxiliary>{
 public:
  BaroUpdateMeasAuxiliary(){
  };
  virtual ~BaroUpdateMeasAuxiliary(){};
};

/**  \brief Empty measurement
 */
class BaroUpdateMeas: public LWF::State<LWF::VectorElement<3>,BaroUpdateMeasAuxiliary>{
 public:
  typedef LWF::State<LWF::VectorElement<3>,BaroUpdateMeasAuxiliary> Base;
  using Base::E_;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _aux = _pos+1;

  BaroUpdateMeas(){
    static_assert(_aux+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_aux>() = "aux";
  };
  virtual ~BaroUpdateMeas(){};
  inline V3D& pos(){
    return this->template get<_pos>();
  }
  inline const V3D& pos() const{
    return this->template get<_pos>();
  }
};


/**  \brief Class holding the update noise.
 */
class BaroUpdateNoise: public LWF::State<LWF::VectorElement<3>>{
 public:
  typedef LWF::State<LWF::VectorElement<3>> Base;
  using Base::E_;
  static constexpr unsigned int _pos = 0;
  BaroUpdateNoise(){
    static_assert(_pos+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
  };
  virtual ~BaroUpdateNoise(){};
  inline V3D& pos(){
    return this->template get<_pos>();
  }
  inline const V3D& pos() const{
    return this->template get<_pos>();
  }
};

/** \brief Outlier Detection.
 * ODEntry<Start entry, dimension of detection>
 */
class BaroOutlierDetection: public LWF::OutlierDetection<LWF::ODEntry<BaroInnovation::template getId<BaroInnovation::_pos>(),3>>{
 public:
  virtual ~BaroOutlierDetection(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding the zero Baro update
 */
template<typename FILTERSTATE, int inertialPoseIndex, int bodyPoseIndex>
class BaroUpdate: public LWF::Update<BaroInnovation,FILTERSTATE,BaroUpdateMeas,
                                         BaroUpdateNoise,BaroOutlierDetection,false>{
 public:
  typedef LWF::Update<BaroInnovation,FILTERSTATE,BaroUpdateMeas,
                      BaroUpdateNoise,BaroOutlierDetection,false> Base;
  using Base::doubleRegister_;
  using Base::intRegister_;
  using Base::meas_;
  using Base::updnoiP_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  static constexpr int inertialPoseIndex_ = inertialPoseIndex;
  static constexpr int bodyPoseIndex_ = bodyPoseIndex;
  QPD qVM_;
  V3D MrMV_;
  QPD qWI_;
  V3D IrIW_;

  Eigen::MatrixXd defaultUpdnoiP_; // Configured update covariance, that will (optionally) be scaled by the measurement

  /** \brief Constructor.
   *
   *   Loads and sets the needed parameters.
   */
  BaroUpdate() : defaultUpdnoiP_((int)(mtNoise::D_),(int)(mtNoise::D_)){
    static_assert(mtState::nPose_>inertialPoseIndex_,"Please add enough poses to the filter state (templated).");
    static_assert(mtState::nPose_>bodyPoseIndex_,"Please add enough poses to the filter state (templated).");
    intRegister_.removeScalarByStr("maxNumIteration");
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    doubleRegister_.removeScalarByStr("updateVecNormTermination");
    
    defaultUpdnoiP_.setZero();

    // Unregister configured covariance
    // Adapter safety patch: the pressure noise has three dimensions.
    for (int i=0;i<mtNoise::D_;i++) {
      doubleRegister_.removeScalarByVar(updnoiP_(i,i));
    }
    // Register configured covariance again under a different name
    mtNoise n;
    n.setIdentity();
    n.registerCovarianceToPropertyHandler_(defaultUpdnoiP_,this,"UpdateNoise.");

  };

  /** \brief Destructor
   */
  virtual ~BaroUpdate(){};
  const V3D& get_IrIW(const mtState& state) const{
    if(inertialPoseIndex_ >= 0){
      return state.poseLin(inertialPoseIndex_);
    } else {
      return IrIW_;
    }
  }
  const V3D& get_MrMV(const mtState& state) const{
    if(bodyPoseIndex_ >= 0){
      return state.poseLin(bodyPoseIndex_);
    } else {
      return MrMV_;
    }
  }

  /** \brief Compute the inovvation term
   *
   *  @param mtInnovation - Class, holding innovation data.
   *  @param state        - Filter %State.
   *  @param meas         - Not Used.
   *  @param noise        - Additive discrete Gaussian noise.
   *  @param dt           - Not used.
   */
  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const{
    y.pos() = get_IrIW(state) + V3D(state.WrWM() + get_MrMV(state)) - meas_.pos() + noise.pos();
  }

  /** \brief Computes the Jacobian for the update step of the filter.
   *
   *  @param F     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   *  @param meas  - Not used.
   *  @param dt    - Not used.
   */
  void jacState(MXD& F, const mtState& state) const{
    F.setZero();
    
    Eigen::Matrix3d jac_z = Eigen::Matrix3d::Identity();

    jac_z(0,0) = 0;
    jac_z(1,1) = 0;

    F.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pos>()) = jac_z;    
  }

  /** \brief Computes the Jacobian for the update step of the filter w.r.t. to the noise variables
   *
   *  @param G     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   *  @param meas  - Not used.
   *  @param dt    - Not used.
   */
  void jacNoise(MXD& G, const mtState& state) const{
    G.setZero();
    
    G.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity();
  }

  void preProcess(mtFilterState& filterstate, const mtMeas& meas, bool& isFinished){
    mtState& state = filterstate.state_;
    isFinished = false;
    // When enabled, scale the configured position covariance by the values in the measurement    
    updnoiP_ = defaultUpdnoiP_;

  }
  void postProcess(mtFilterState& filterstate, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    mtState& state = filterstate.state_;
    isFinished = true;
  }

};

}

#endif /* ROVIO_BaroUPDATE_HPP_ */
