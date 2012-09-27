class TestmodelsController < ApplicationController
  # GET /testmodels
  # GET /testmodels.json
  def index
    @testmodels = Testmodel.all

    respond_to do |format|
      format.html # index.html.erb
      format.json { render json: @testmodels }
    end
  end

  # GET /testmodels/1
  # GET /testmodels/1.json
  def show
    @testmodel = Testmodel.find(params[:id])

    respond_to do |format|
      format.html # show.html.erb
      format.json { render json: @testmodel }
    end
  end

  # GET /testmodels/new
  # GET /testmodels/new.json
  def new
    @testmodel = Testmodel.new

    respond_to do |format|
      format.html # new.html.erb
      format.json { render json: @testmodel }
    end
  end

  # GET /testmodels/1/edit
  def edit
    @testmodel = Testmodel.find(params[:id])
  end

  # POST /testmodels
  # POST /testmodels.json
  def create
    @testmodel = Testmodel.new(params[:testmodel])

    respond_to do |format|
      if @testmodel.save
        format.html { redirect_to @testmodel, notice: 'Testmodel was successfully created.' }
        format.json { render json: @testmodel, status: :created, location: @testmodel }
      else
        format.html { render action: "new" }
        format.json { render json: @testmodel.errors, status: :unprocessable_entity }
      end
    end
  end

  # PUT /testmodels/1
  # PUT /testmodels/1.json
  def update
    @testmodel = Testmodel.find(params[:id])

    respond_to do |format|
      if @testmodel.update_attributes(params[:testmodel])
        format.html { redirect_to @testmodel, notice: 'Testmodel was successfully updated.' }
        format.json { head :no_content }
      else
        format.html { render action: "edit" }
        format.json { render json: @testmodel.errors, status: :unprocessable_entity }
      end
    end
  end

  # DELETE /testmodels/1
  # DELETE /testmodels/1.json
  def destroy
    @testmodel = Testmodel.find(params[:id])
    @testmodel.destroy

    respond_to do |format|
      format.html { redirect_to testmodels_url }
      format.json { head :no_content }
    end
  end
end
