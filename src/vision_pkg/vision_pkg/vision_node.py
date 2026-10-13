#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import easyocr

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String


class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_node")

        self.bridge = CvBridge()

        # ============================================================
        # EasyOCR
        # ============================================================

        self.get_logger().info("Loading EasyOCR...")

        self.reader = easyocr.Reader(
            ['en'],
            gpu=False
        )

        self.get_logger().info("EasyOCR loaded")

        # ============================================================
        # Subscribers
        # ============================================================

        self.letter_subscription = self.create_subscription(
            Image,
            "/letter_image",
            self.letter_callback,
            10
        )

        self.number_subscription = self.create_subscription(
            Image,
            "/number_image",
            self.number_callback,
            10
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.publisher = self.create_publisher(
            String,
            "/vision_cell",
            10
        )

        # ============================================================
        # Stored results
        # ============================================================

        self.last_letter = ""
        self.last_number = ""

        # ============================================================
        # OpenCV windows
        # ============================================================

        cv.namedWindow(
            "Letter Original",
            cv.WINDOW_NORMAL
        )

        cv.namedWindow(
            "Letter OCR",
            cv.WINDOW_NORMAL
        )

        cv.namedWindow(
            "Number Original",
            cv.WINDOW_NORMAL
        )

        cv.namedWindow(
            "Number OCR",
            cv.WINDOW_NORMAL
        )

        cv.resizeWindow(
            "Letter Original",
            600,
            450
        )

        cv.resizeWindow(
            "Letter OCR",
            500,
            500
        )

        cv.resizeWindow(
            "Number Original",
            600,
            450
        )

        cv.resizeWindow(
            "Number OCR",
            500,
            500
        )

        self.get_logger().info(
            "Vision Node Started"
        )

        self.get_logger().info(
            "Waiting for /letter_image and /number_image"
        )

    # ================================================================
    # LETTER CALLBACK
    # ================================================================

    def letter_callback(self, msg):

        try:

            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            self.get_logger().info(
                "Received letter image"
            )

            letter = self.recognize_letter(
                image
            )

            if letter:

                self.last_letter = letter

                self.get_logger().info(
                    f"Letter = {letter}"
                )

                self.try_publish_cell()

            else:

                self.get_logger().warn(
                    "Could not recognize letter"
                )

        except Exception as e:

            self.get_logger().error(
                f"Letter processing error: {e}"
            )

    # ================================================================
    # NUMBER CALLBACK
    # ================================================================

    def number_callback(self, msg):

        try:

            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            self.get_logger().info(
                "Received number image"
            )

            number = self.recognize_number(
                image
            )

            if number:

                self.last_number = number

                self.get_logger().info(
                    f"Number = {number}"
                )

                self.try_publish_cell()

            else:

                self.get_logger().warn(
                    "Could not recognize number"
                )

        except Exception as e:

            self.get_logger().error(
                f"Number processing error: {e}"
            )

    # ================================================================
    # PREPARE IMAGE FOR OCR
    # ================================================================

    def prepare_for_ocr(self, image):

        if image is None or image.size == 0:
            return None

        h, w = image.shape[:2]

        # ------------------------------------------------------------
        # Upscale small images
        # ------------------------------------------------------------

        if max(h, w) < 250:

            image = cv.resize(
                image,
                None,
                fx=3.0,
                fy=3.0,
                interpolation=cv.INTER_CUBIC
            )

        elif max(h, w) < 500:

            image = cv.resize(
                image,
                None,
                fx=2.0,
                fy=2.0,
                interpolation=cv.INTER_CUBIC
            )

        # ------------------------------------------------------------
        # Add border
        # ------------------------------------------------------------

        image = cv.copyMakeBorder(
            image,
            40,
            40,
            40,
            40,
            cv.BORDER_CONSTANT,
            value=(255, 255, 255)
        )

        return image

    # ================================================================
    # CREATE LETTER OCR VERSIONS
    #
    # Based on your test results:
    #
    # 1. Original
    # 2. Grayscale
    # 3. OTSU
    # 4. OTSU Inverted
    #
    # Enhanced / Adaptive versions removed.
    # ================================================================

    def create_letter_versions(self, image):

        versions = []

        # ------------------------------------------------------------
        # 1. Original
        # ------------------------------------------------------------

        original = self.prepare_for_ocr(
            image.copy()
        )

        if original is not None:

            versions.append(
                ("Original", original)
            )

        # ------------------------------------------------------------
        # Grayscale
        # ------------------------------------------------------------

        gray = cv.cvtColor(
            image,
            cv.COLOR_BGR2GRAY
        )

        gray_prepared = self.prepare_for_ocr(
            gray
        )

        if gray_prepared is not None:

            versions.append(
                ("Grayscale", gray_prepared)
            )

        # ------------------------------------------------------------
        # OTSU
        # ------------------------------------------------------------

        _, otsu = cv.threshold(
            gray,
            0,
            255,
            cv.THRESH_BINARY + cv.THRESH_OTSU
        )

        otsu = self.prepare_for_ocr(
            otsu
        )

        if otsu is not None:

            versions.append(
                ("OTSU", otsu)
            )

        # ------------------------------------------------------------
        # OTSU Inverted
        # ------------------------------------------------------------

        _, otsu_inv = cv.threshold(
            gray,
            0,
            255,
            cv.THRESH_BINARY_INV + cv.THRESH_OTSU
        )

        otsu_inv = self.prepare_for_ocr(
            otsu_inv
        )

        if otsu_inv is not None:

            versions.append(
                ("OTSU Inverted", otsu_inv)
            )

        return versions

    # ================================================================
    # CREATE NUMBER OCR VERSIONS
    #
    # Based on your test results:
    #
    # 1. Original
    # 2. OTSU
    # 3. Grayscale
    # 4. OTSU Inverted
    #
    # Enhanced / Adaptive versions removed.
    # ================================================================

    def create_number_versions(self, image):

        versions = []

        # ------------------------------------------------------------
        # 1. Original
        # ------------------------------------------------------------

        original = self.prepare_for_ocr(
            image.copy()
        )

        if original is not None:

            versions.append(
                ("Original", original)
            )

        # ------------------------------------------------------------
        # Grayscale
        # ------------------------------------------------------------

        gray = cv.cvtColor(
            image,
            cv.COLOR_BGR2GRAY
        )

        # ------------------------------------------------------------
        # OTSU
        #
        # Put OTSU before grayscale for numbers because your test
        # showed OTSU was extremely reliable.
        # ------------------------------------------------------------

        _, otsu = cv.threshold(
            gray,
            0,
            255,
            cv.THRESH_BINARY + cv.THRESH_OTSU
        )

        otsu = self.prepare_for_ocr(
            otsu
        )

        if otsu is not None:

            versions.append(
                ("OTSU", otsu)
            )

        # ------------------------------------------------------------
        # Grayscale
        # ------------------------------------------------------------

        gray_prepared = self.prepare_for_ocr(
            gray
        )

        if gray_prepared is not None:

            versions.append(
                ("Grayscale", gray_prepared)
            )

        # ------------------------------------------------------------
        # OTSU Inverted
        # ------------------------------------------------------------

        _, otsu_inv = cv.threshold(
            gray,
            0,
            255,
            cv.THRESH_BINARY_INV + cv.THRESH_OTSU
        )

        otsu_inv = self.prepare_for_ocr(
            otsu_inv
        )

        if otsu_inv is not None:

            versions.append(
                ("OTSU Inverted", otsu_inv)
            )

        return versions

    # ================================================================
    # RUN OCR FOR LETTER
    # ================================================================

    def recognize_letter(self, image):

        if image is None or image.size == 0:

            self.get_logger().warn(
                "Empty letter image"
            )

            return ""

        # ------------------------------------------------------------
        # Show original
        # ------------------------------------------------------------

        cv.imshow(
            "Letter Original",
            image
        )

        cv.waitKey(1)

        height, width = image.shape[:2]

        self.get_logger().info(
            f"Letter image size: {width}x{height}"
        )

        # ------------------------------------------------------------
        # Create OCR versions
        # ------------------------------------------------------------

        versions = self.create_letter_versions(
            image
        )

        if not versions:

            self.get_logger().warn(
                "Could not create letter OCR versions"
            )

            return ""

        # ------------------------------------------------------------
        # Try each version
        # ------------------------------------------------------------

        for version_name, ocr_image in versions:

            self.get_logger().info(
                f"Running EasyOCR on letter: "
                f"{version_name}"
            )

            cv.imshow(
                "Letter OCR",
                ocr_image
            )

            cv.waitKey(1)

            # --------------------------------------------------------
            # Save image
            # --------------------------------------------------------

            safe_name = (
                version_name
                .lower()
                .replace(" ", "_")
            )

            cv.imwrite(
                f"/tmp/letter_{safe_name}.png",
                ocr_image
            )

            # --------------------------------------------------------
            # EasyOCR
            # --------------------------------------------------------

            try:

                results = self.reader.readtext(
                    ocr_image,
                    detail=1,
                    paragraph=False,
                    allowlist="ABCDEFGHIJKLMNOPQRSTUVWXYZ",
                    mag_ratio=2.0,
                    text_threshold=0.4,
                    low_text=0.2,
                    link_threshold=0.2
                )

            except Exception as e:

                self.get_logger().error(
                    f"EasyOCR letter error on "
                    f"{version_name}: {e}"
                )

                continue

            self.get_logger().info(
                f"EasyOCR letter {version_name}: "
                f"{results}"
            )

            # --------------------------------------------------------
            # Check results
            # --------------------------------------------------------

            for _, text, confidence in results:

                text = text.strip().upper()

                self.get_logger().info(
                    f"Letter OCR {version_name}: "
                    f"'{text}' "
                    f"confidence={confidence:.3f}"
                )

                # ----------------------------------------------------
                # Must be exactly one letter
                # ----------------------------------------------------

                if len(text) != 1:
                    continue

                # ----------------------------------------------------
                # Only A-S
                # ----------------------------------------------------

                if not (
                    "A" <= text <= "S"
                ):
                    continue

                # ----------------------------------------------------
                # High confidence
                #
                # Stop immediately.
                # ----------------------------------------------------

                if confidence >= 0.80:

                    self.get_logger().info(
                        f"Letter accepted: {text} "
                        f"confidence={confidence:.3f} "
                        f"version={version_name}"
                    )

                    return text

        # ------------------------------------------------------------
        # Nothing passed the confidence threshold
        # ------------------------------------------------------------

        self.get_logger().warn(
            "No high-confidence letter found"
        )

        return ""

    # ================================================================
    # RUN OCR FOR NUMBER
    # ================================================================

    def recognize_number(self, image):

        if image is None or image.size == 0:

            self.get_logger().warn(
                "Empty number image"
            )

            return ""

        # ------------------------------------------------------------
        # Show original
        # ------------------------------------------------------------

        cv.imshow(
            "Number Original",
            image
        )

        cv.waitKey(1)

        height, width = image.shape[:2]

        self.get_logger().info(
            f"Number image size: {width}x{height}"
        )

        # ------------------------------------------------------------
        # Create OCR versions
        # ------------------------------------------------------------

        versions = self.create_number_versions(
            image
        )

        if not versions:

            self.get_logger().warn(
                "Could not create number OCR versions"
            )

            return ""

        # ------------------------------------------------------------
        # Try each version
        # ------------------------------------------------------------

        for version_name, ocr_image in versions:

            self.get_logger().info(
                f"Running EasyOCR on number: "
                f"{version_name}"
            )

            cv.imshow(
                "Number OCR",
                ocr_image
            )

            cv.waitKey(1)

            # --------------------------------------------------------
            # Save image
            # --------------------------------------------------------

            safe_name = (
                version_name
                .lower()
                .replace(" ", "_")
            )

            cv.imwrite(
                f"/tmp/number_{safe_name}.png",
                ocr_image
            )

            # --------------------------------------------------------
            # EasyOCR
            # --------------------------------------------------------

            try:

                results = self.reader.readtext(
                    ocr_image,
                    detail=1,
                    paragraph=False,
                    allowlist="0123456789",
                    mag_ratio=2.0,
                    text_threshold=0.4,
                    low_text=0.2,
                    link_threshold=0.2
                )

            except Exception as e:

                self.get_logger().error(
                    f"EasyOCR number error on "
                    f"{version_name}: {e}"
                )

                continue

            self.get_logger().info(
                f"EasyOCR number {version_name}: "
                f"{results}"
            )

            # --------------------------------------------------------
            # Check results
            # --------------------------------------------------------

            for _, text, confidence in results:

                text = text.strip()

                self.get_logger().info(
                    f"Number OCR {version_name}: "
                    f"'{text}' "
                    f"confidence={confidence:.3f}"
                )

                # ----------------------------------------------------
                # Keep digits only
                # ----------------------------------------------------

                cleaned_text = "".join(
                    ch for ch in text
                    if ch.isdigit()
                )

                if not cleaned_text:
                    continue

                # ----------------------------------------------------
                # Maximum two digits
                # ----------------------------------------------------

                if len(cleaned_text) > 2:
                    continue

                # ----------------------------------------------------
                # Convert to integer
                # ----------------------------------------------------

                try:

                    value = int(
                        cleaned_text
                    )

                except ValueError:

                    continue

                # ----------------------------------------------------
                # Competition range
                # ----------------------------------------------------

                if value < 1 or value > 19:
                    continue

                # ----------------------------------------------------
                # High confidence
                #
                # Stop immediately.
                # ----------------------------------------------------

                if confidence >= 0.80:

                    self.get_logger().info(
                        f"Number accepted: {cleaned_text} "
                        f"confidence={confidence:.3f} "
                        f"version={version_name}"
                    )

                    return cleaned_text

        # ------------------------------------------------------------
        # Nothing passed the confidence threshold
        # ------------------------------------------------------------

        self.get_logger().warn(
            "No high-confidence number found"
        )

        return ""

    # ================================================================
    # VALIDATE CELL
    # ================================================================

    def valid_cell(self, cell):

        if len(cell) < 2:
            return False

        letter = "".join(
            ch for ch in cell
            if ch.isalpha()
        )

        number = "".join(
            ch for ch in cell
            if ch.isdigit()
        )

        if len(letter) != 1:
            return False

        if letter < "A" or letter > "S":
            return False

        if not number:
            return False

        value = int(number)

        if value < 1 or value > 19:
            return False

        return True

    # ================================================================
    # COMBINE LETTER + NUMBER
    # ================================================================

    def try_publish_cell(self):

        if not self.last_letter:
            return

        if not self.last_number:
            return

        cell = (
            self.last_letter
            +
            self.last_number
        )

        self.get_logger().info(
            f"Cell = {cell}"
        )

        if not self.valid_cell(cell):

            self.get_logger().warn(
                f"Invalid cell: {cell}"
            )

            return

        msg = String()

        msg.data = cell

        self.publisher.publish(
            msg
        )

        self.get_logger().info(
            f"Published /vision_cell = {cell}"
        )

        # ------------------------------------------------------------
        # Clear after publishing
        # ------------------------------------------------------------

        self.last_letter = ""
        self.last_number = ""

    # ================================================================
    # CLEANUP
    # ================================================================

    def destroy_node(self):

        cv.destroyAllWindows()

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = VisionNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()