// This file is part of the Acts project.
//
// Copyright (C) 2019 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "Acts/EventData/ParameterConcept.hpp"

template <typename S, typename N>
template <typename result_t, typename propagator_state_t>
auto Acts::Propagator<S, N>::propagate_impl(propagator_state_t& state) const
    -> Result<result_t> {
  result_t result;
  std::cout<<"Joe: entering propagate_impl"<<std::endl;
  // Pre-stepping call to the navigator and action list
  debugLog(state, [&] { return std::string("Entering propagation."); });
 
  // Navigator initialize state call
  m_navigator.status(state, m_stepper);
  std::cout<<"Joe: pre-stepping call to action list"<<std::endl;
  // Pre-Stepping call to the action list
  state.options.actionList(state, m_stepper, result);
  std::cout<<"Joe: post-stepping call to action list"<<std::endl;
  // assume negative outcome, only set to true later if we actually have
  // a positive outcome.
  // This is needed for correct error logging
  bool terminatedNormally = false;
  // Pre-Stepping: abort condition check
  if (!state.options.abortList(result, state, m_stepper)) {
    // Pre-Stepping: target setting
    std::cout<<"Joe: navigator target"<<std::endl;
    m_navigator.target(state, m_stepper);
    // Stepping loop
    debugLog(state, [&] { return std::string("Starting stepping loop."); });
    // Propagation loop : stepping
    std::cout<<"Joe: stepping loop"<<std::endl;
    for (; result.steps < state.options.maxSteps; ++result.steps) {
      // Perform a propagation step - it takes the propagation state
      std::cout<<"Joe: Step the state"<<std::endl;
      std::cout<<"Joe: Stepping state : "<<std::endl;
      std::cout<<"Joe : "<<state.stepping.pos<<std::endl;
      Result<double> res = m_stepper.step(state);
      std::cout<<"Joe: state stepped"<<std::endl;
      if (res.ok()) {
        // Accumulate the path length
        double s = *res;
        result.pathLength += s;
	std::cout<<"Joe: State path length accumulated: " << s << std::endl;
        debugLog(state, [&] {
          std::stringstream dstream;
          dstream << "Step with size = ";
          dstream << s;
          dstream << " performed.";
          return dstream.str();
        });
      } else {
        debugLog(state, [&] {
          std::stringstream dstream;
          dstream << "Step failed: ";
          dstream << res.error();
          return dstream.str();
        });
        // pass error to caller
        return res.error();
      }
      // Post-stepping:
      // navigator status call - action list - aborter list - target call
            
      std::cout<<"Joe: post state stepping"<<std::endl;
      m_navigator.status(state, m_stepper);
      std::cout<<"Joe: navigator status"<<std::endl;
      state.options.actionList(state, m_stepper, result);
      std::cout<<"Joe: Perform next Actor action"<<std::endl;
      if (state.options.abortList(result, state, m_stepper)) {
      std::cout<<"Joe: Terminated aborter normally"<<std::endl;
        terminatedNormally = true;
        break;
      }
     std::cout<<"Joe: navigator target"<<std::endl;   
     m_navigator.target(state, m_stepper);
     std::cout<<"Joe: set navigator new target"<<std::endl;
    }
  }
  std::cout<<"Joe: check if normal termination for state stepper"<<std::endl;
  // if we didn't terminate normally (via aborters) set navigation break.
  // this will trigger error output in the lines below
  if (!terminatedNormally) {
    debugLog(state, [&] { return std::string("Terminated with failure."); });
    state.navigation.navigationBreak = true;
  }
  std::cout<<"Joe: post stepping call to action list. Stepping loop done"<<std::endl;
  // Post-stepping call to the action list
  debugLog(state, [&] { return std::string("Stepping loop done."); });
  state.options.actionList(state, m_stepper, result);
  std::cout<<"Joe: return progress flag"<<std::endl;
  // return progress flag here, decide on SUCCESS later
  return std::move(result);
}

template <typename S, typename N>
template <typename parameters_t, typename propagator_options_t,
          typename path_aborter_t>
auto Acts::Propagator<S, N>::propagate(
    const parameters_t& start, const propagator_options_t& options) const
    -> Result<action_list_t_result_t<
        CurvilinearParameters,
        typename propagator_options_t::action_list_type>> {
  static_assert(ParameterConcept<parameters_t>,
                "Parameters do not fulfill parameter concept.");

  std::cout<<"Joe: Starting propagation"<<std::endl;
  // Type of track parameters produced by the propagation
  using ReturnParameterType = CurvilinearParameters;

  // Type of the full propagation result, including output from actions
  using ResultType =
      action_list_t_result_t<ReturnParameterType,
                             typename propagator_options_t::action_list_type>;

  static_assert(std::is_copy_constructible<ReturnParameterType>::value,
                "return track parameter type must be copy-constructible");

  std::cout<<"Joe: Expand abort list"<<std::endl;
  // Expand the abort list with a path aborter
  path_aborter_t pathAborter;
  pathAborter.internalLimit = options.pathLimit;

  std::cout<<"Joe: abortList.append"<<std::endl;
  auto abortList = options.abortList.append(pathAborter);

  std::cout<<"Joe: extend abort list"<<std::endl;
  // The expanded options (including path limit)
  auto eOptions = options.extend(abortList);
  using OptionsType = decltype(eOptions);
  // Initialize the internal propagator state
  using StateType = State<OptionsType>;
  std::cout<<"Joe: create StateType"<<std::endl;
  StateType state(start, eOptions);

  static_assert(
      concept ::has_method<const S, Result<double>, concept ::Stepper::step_t,
                           StateType&>,
      "Step method of the Stepper is not compatible with the propagator "
      "state");

  std::cout<<"Joe: apply loop protection"<<std::endl;
  // Apply the loop protection - it resets the internal path limit
  if (options.loopProtection) {
    detail::LoopProtection<path_aborter_t> lProtection;
    lProtection(state, m_stepper);
  }
  std::cout<<"Joe: Perform propagation"<<std::endl;
  // Perform the actual propagation & check its outcome
  auto result = propagate_impl<ResultType>(state);
  std::cout<<"Joe: Check result"<<std::endl;
  if (result.ok()) {
    std::cout<<"Joe: result okay"<<std::endl;
    auto& propRes = *result;
    /// Convert into return type and fill the result object
    std::cout<<"Joe: fill prop result"<<std::endl;
    auto curvState = m_stepper.curvilinearState(state.stepping);
    auto& curvParameters = std::get<CurvilinearParameters>(curvState);
    // Fill the end parameters
    propRes.endParameters = std::make_unique<const CurvilinearParameters>(
        std::move(curvParameters));
    // Only fill the transport jacobian when covariance transport was done
    if (state.stepping.covTransport) {
      auto& tJacobian = std::get<Jacobian>(curvState);
      propRes.transportJacobian =
          std::make_unique<const Jacobian>(std::move(tJacobian));
    }
    std::cout<<"Joe: return propagation result"<<std::endl;
    return result;
  } else {
    return result.error();
  }
}

template <typename S, typename N>
template <typename parameters_t, typename propagator_options_t,
          typename target_aborter_t, typename path_aborter_t>
auto Acts::Propagator<S, N>::propagate(
    const parameters_t& start, const Surface& target,
    const propagator_options_t& options) const
    -> Result<action_list_t_result_t<
        BoundParameters, typename propagator_options_t::action_list_type>> {
  static_assert(ParameterConcept<parameters_t>,
                "Parameters do not fulfill parameter concept.");

  // Type of track parameters produced at the end of the propagation
  using return_parameter_type = BoundParameters;

  // Type of provided options
  target_aborter_t targetAborter;
  path_aborter_t pathAborter;
  pathAborter.internalLimit = options.pathLimit;
  auto abortList = options.abortList.append(targetAborter, pathAborter);

  // Create the extended options and declare their type
  auto eOptions = options.extend(abortList);
  using OptionsType = decltype(eOptions);

  // Type of the full propagation result, including output from actions
  using ResultType =
      action_list_t_result_t<return_parameter_type,
                             typename propagator_options_t::action_list_type>;

  // Initialize the internal propagator state
  using StateType = State<OptionsType>;
  StateType state(start, eOptions);
  state.navigation.targetSurface = &target;

  static_assert(
      concept ::has_method<const S, Result<double>, concept ::Stepper::step_t,
                           StateType&>,
      "Step method of the Stepper is not compatible with the propagator "
      "state");

  // Apply the loop protection, it resets the interal path limit
  detail::LoopProtection<path_aborter_t> lProtection;
  lProtection(state, m_stepper);

  // Perform the actual propagation
  auto result = propagate_impl<ResultType>(state);

  if (result.ok()) {
    auto& propRes = *result;
    // Compute the final results and mark the propagation as successful
    auto bs = m_stepper.boundState(state.stepping, target);
    auto& boundParameters = std::get<BoundParameters>(bs);
    // Fill the end parameters
    propRes.endParameters =
        std::make_unique<const BoundParameters>(std::move(boundParameters));
    // Only fill the transport jacobian when covariance transport was done
    if (state.stepping.covTransport) {
      auto& tJacobian = std::get<Jacobian>(bs);
      propRes.transportJacobian =
          std::make_unique<const Jacobian>(std::move(tJacobian));
    }
    return result;
  } else {
    return result.error();
  }
}

template <typename S, typename N>
template <typename propagator_state_t>
void Acts::Propagator<S, N>::debugLog(
    propagator_state_t& state,
    const std::function<std::string()>& logAction) const {
  if (state.options.debug) {
    std::vector<std::string> lines;
    std::string input = logAction();
    boost::split(lines, input, boost::is_any_of("\n"));
    for (const auto& line : lines) {
      std::stringstream dstream;
      dstream << "|->" << std::setw(state.options.debugPfxWidth);
      dstream << "Propagator"
              << " | ";
      dstream << std::setw(state.options.debugMsgWidth) << line << '\n';
      state.options.debugString += dstream.str();
    }
  }
}