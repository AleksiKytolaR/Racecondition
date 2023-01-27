/* eslint-disable @typescript-eslint/no-unused-vars */
type Frame = Pixel[][];
type Pixel = { r: number; g: number; b: number };
const RED: Pixel = { r: 255, g: 0, b: 0 };
const GREEN: Pixel = { r: 0, g: 255, b: 0 };
const BLUE: Pixel = { r: 0, g: 0, b: 255 };
const BLACK: Pixel = { r: 0, g: 0, b: 0 };

/**
* Pixel frame preprocessor. The example implementation simplifies each pixel
to either RED, GREEN, BLUE or BLACK. Your implementation may do something more
advanced, including changing the dimensions of this frame. The result always
needs to be a 2 dimensional array of RGB color objects though.
**/
function preprocess(frame: Frame) {
	const threshold = 10;
	return frame.map((row) =>
		row.map((pixel) => {
			if (pixel.r - Math.max(pixel.g, pixel.b) > threshold) return RED;
			if (pixel.g - Math.max(pixel.r, pixel.b) > threshold) return GREEN;
			if (pixel.b - Math.max(pixel.g, pixel.r) > threshold) return BLUE;
			return BLACK;
		})
	);
}

function sameColor(x: Pixel, y: Pixel) {
	return x.r === y.r && x.g === y.g && x.b === y.b;
}

function decide(frame: Frame) {
	// Choose the center row
	const height = frame.length;
	const bottomRow = frame[height - 1];
	// Choose a pixel at the center of this row
	const width = bottomRow.length;
	const pixel = bottomRow[width / 2];
	if (sameColor(pixel, RED)) {
		// Red side of track -> turn right
		return { throttle: 0.1, steering: -1 };
	}
	return { throttle: 0.1, steering: 0 };
}
