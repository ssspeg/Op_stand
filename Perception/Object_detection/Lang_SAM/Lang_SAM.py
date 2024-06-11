import os
os.environ["HF_ENDPOINT"] = "https://hf-mirror.com"
import numpy as np
from PIL import Image
from typing import List, Type, Tuple
# from image_processors.image_processor import ImageProcessor
from lang_sam import LangSAM
from Perception.Object_detection import object_detection


class Lang_SAM(object_detection):
    
    def __init__(self):
        
        self.model = LangSAM()

    def detect_obj(
        self,
        image: Type[Image.Image],
        text: str = None,
        bbox: List[int] = None,
        save_box: bool = False,
        box_filename: str = None,
        save_mask: bool = False,
        mask_filename: str = None,
    ) -> Tuple[np.ndarray, List[int]]:
        """
        Args:
            image: An image object on which object detection is performed.
            text: Optional parameter for performing text-related object detection tasks, a object name in the scence, eg. "cup". Defaults to None.
            bbox: Optional parameter specifying an initial bounding box. Defaults to None.
            save_box: Optional parameter indicating whether to save bounding boxes. Defaults to False.
            box_filename: Optional parameter specifying the filename to save the visualization of bounding boxes. Defaults to None.
            save_mask: Optional parameter indicating whether to save masks. Defaults to False.
            mask_filename: Optional parameter specifying the filename to save the visualization of masks. Defaults to None.
            
        Returns:
            seg_mask: the segmentation mask of the detected object in the input image.
            bbox: the bounding box coordinates of the detected object in the input image
        """
        
        masks, boxes, phrases, logits = self.model.predict(image, text)
        if len(masks) == 0:
            return masks, None

        seg_mask = np.array(masks[0])
        bbox = np.array(boxes[0], dtype=int)

        if save_box:
            self.draw_bounding_box(image, bbox, box_filename)

        if save_mask:
            self.draw_mask_on_image(image, seg_mask, mask_filename)

        return seg_mask, bbox
    

if __name__=='__main__':
    
    # set work dir to Lang-SAM
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    lang_sam = Lang_SAM()
    
    image = Image.open(f"./test_image/test_rgb.jpg")
    query = str(input("Enter a Object name in the image: "))
    box_filename = f"./output/object_box.jpg"
    mask_filename = f"./output/object_mask.jpg"
    
    # Object Segmentaion Mask
    seg_mask, bbox = lang_sam.detect_obj(
        image,
        query,
        save_box=True,
        save_mask=True,
        box_filename=box_filename,
        mask_filename=mask_filename
    )