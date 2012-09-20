class CreateRobotStates < ActiveRecord::Migration
  def change
    create_table :robot_states do |t|
      t.string :user
      t.string :activity
      t.references :robots

      t.timestamps
    end
    add_index :robot_states, :robots_id
  end
end
