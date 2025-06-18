import os
import shutil
import datetime

class PhotoManager:
    def __init__(self, photo_dir='photos', upload_dir='uploaded'):
        self.photo_dir = photo_dir
        self.upload_dir = upload_dir
        os.makedirs(self.photo_dir, exist_ok=True)
        os.makedirs(self.upload_dir, exist_ok=True)

    def generate_filename(self, prefix='photo', ext='jpg'):
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"{prefix}_{timestamp}.{ext}"
        return filename

    def save_photo(self, photo_data, prefix='photo', ext='jpg'):
        filename = self.generate_filename(prefix, ext)
        filepath = os.path.join(self.photo_dir, filename)
        with open(filepath, 'wb') as f:
            f.write(photo_data)
        return filepath

    def organize_photos_by_date(self):
        for fname in os.listdir(self.photo_dir):
            if not fname.lower().endswith(('.jpg', '.jpeg', '.png')):
                continue
            date_part = fname.split('_')[1] if '_' in fname else 'unknown'
            date_folder = os.path.join(self.photo_dir, date_part)
            os.makedirs(date_folder, exist_ok=True)
            src = os.path.join(self.photo_dir, fname)
            dst = os.path.join(date_folder, fname)
            shutil.move(src, dst)

    def upload_photo(self, filepath):
        # Simulate upload by moving to upload_dir
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"{filepath} does not exist")
        fname = os.path.basename(filepath)
        dst = os.path.join(self.upload_dir, fname)
        shutil.move(filepath, dst)
        return dst

# Example usage:
if __name__ == "__main__":
    manager = PhotoManager()
    # Simulate photo data
    photo_data = b'\xff\xd8\xff\xe0'  # JPEG header bytes
    saved_path = manager.save_photo(photo_data)
    print(f"Photo saved to: {saved_path}")
    manager.organize_photos_by_date()
    # Find a photo to upload
    for root, dirs, files in os.walk(manager.photo_dir):
        for file in files:
            if file.endswith('.jpg'):
                photo_path = os.path.join(root, file)
                uploaded_path = manager.upload_photo(photo_path)
                print(f"Photo uploaded to: {uploaded_path}")