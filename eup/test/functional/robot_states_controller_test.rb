require 'test_helper'

class RobotStatesControllerTest < ActionController::TestCase
  setup do
    @robot_state = robot_states(:one)
  end

  test "should get index" do
    get :index
    assert_response :success
    assert_not_nil assigns(:robot_states)
  end

  test "should get new" do
    get :new
    assert_response :success
  end

  test "should create robot_state" do
    assert_difference('RobotState.count') do
      post :create, robot_state: { activity: @robot_state.activity, user: @robot_state.user }
    end

    assert_redirected_to robot_state_path(assigns(:robot_state))
  end

  test "should show robot_state" do
    get :show, id: @robot_state
    assert_response :success
  end

  test "should get edit" do
    get :edit, id: @robot_state
    assert_response :success
  end

  test "should update robot_state" do
    put :update, id: @robot_state, robot_state: { activity: @robot_state.activity, user: @robot_state.user }
    assert_redirected_to robot_state_path(assigns(:robot_state))
  end

  test "should destroy robot_state" do
    assert_difference('RobotState.count', -1) do
      delete :destroy, id: @robot_state
    end

    assert_redirected_to robot_states_path
  end
end
