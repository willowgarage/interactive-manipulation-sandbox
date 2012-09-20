class AddAttachmentImageToRobots < ActiveRecord::Migration
  def self.up
    change_table :robots do |t|
      t.has_attached_file :image
    end
  end

  def self.down
    drop_attached_file :robots, :image
  end
end
