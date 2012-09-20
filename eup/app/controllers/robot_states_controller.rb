class RobotStatesController < ApplicationController
  # GET /robot_states
  # GET /robot_states.json
  def index
    @robot_states = RobotState.all

    respond_to do |format|
      format.html # index.html.erb
      format.json { render json: @robot_states }
    end
  end

  # GET /robot_states/1
  # GET /robot_states/1.json
  def show
    @robot_state = RobotState.find(params[:id])

    respond_to do |format|
      format.html # show.html.erb
      format.json { render json: @robot_state }
    end
  end

  # GET /robot_states/new
  # GET /robot_states/new.json
  def new
    @robot_state = RobotState.new

    respond_to do |format|
      format.html # new.html.erb
      format.json { render json: @robot_state }
    end
  end

  # GET /robot_states/1/edit
  def edit
    @robot_state = RobotState.find(params[:id])
  end

  # POST /robot_states
  # POST /robot_states.json
  def create
    @robot_state = RobotState.new(params[:robot_state])

    respond_to do |format|
      if @robot_state.save
        format.html { redirect_to @robot_state, notice: 'Robot state was successfully created.' }
        format.json { render json: @robot_state, status: :created, location: @robot_state }
      else
        format.html { render action: "new" }
        format.json { render json: @robot_state.errors, status: :unprocessable_entity }
      end
    end
  end

  # PUT /robot_states/1
  # PUT /robot_states/1.json
  def update
    @robot_state = RobotState.find(params[:id])

    respond_to do |format|
      if @robot_state.update_attributes(params[:robot_state])
        format.html { redirect_to @robot_state, notice: 'Robot state was successfully updated.' }
        format.json { head :no_content }
      else
        format.html { render action: "edit" }
        format.json { render json: @robot_state.errors, status: :unprocessable_entity }
      end
    end
  end

  # DELETE /robot_states/1
  # DELETE /robot_states/1.json
  def destroy
    @robot_state = RobotState.find(params[:id])
    @robot_state.destroy

    respond_to do |format|
      format.html { redirect_to robot_states_url }
      format.json { head :no_content }
    end
  end
end
