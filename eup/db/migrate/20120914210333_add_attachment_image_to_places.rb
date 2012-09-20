class AddAttachmentImageToPlaces < ActiveRecord::Migration
  def self.up
    change_table :places do |t|
      t.has_attached_file :image
    end
  end

  def self.down
    drop_attached_file :places, :image
  end
end
