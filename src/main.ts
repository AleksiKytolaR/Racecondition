/* eslint-disable @typescript-eslint/no-non-null-assertion */
/* eslint-disable @typescript-eslint/no-unused-vars */
type Frame = Color[][];
type Color = { r: number; g: number; b: number };
type Vec2 = { x: number; y: number };
type Pixel = Vec2 & Color;
type PixelList = Pixel[];
const RED: Color = { r: 255, g: 0, b: 0 };
const GREEN: Color = { r: 0, g: 255, b: 0 };
const BLUE: Color = { r: 0, g: 0, b: 255 };
const BLACK: Color = { r: 0, g: 0, b: 0 };

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

function sameColor(x: Color, y: Color) {
	return x.r === y.r && x.g === y.g && x.b === y.b;
}

function detectBlobs(frame: Frame, minBlobSize = 50) {
	const h = frame.length;
	const w = frame[0].length;
	const blobs: Set<Pixel>[] = [];
	const pixelList = frameToPixelList(frame);
	console.log(`W: ${w}, H: ${h} Count: ${w * h} PixList: ${pixelList.length}`);

	while (pixelList.length > 0) {
		const current = pixelList[0];
		const blob = detectBlobFromPoint(pixelList, current);
		if (blob.size < minBlobSize) continue;
		blobs.push(blob);
	}
	return blobs;
}

function detectBlobFromPoint(pixelList: PixelList, fromPixel: Pixel) {
	const open = new Set<Pixel>();
	const blob = new Set<Pixel>();
	const blobColor = fromPixel;

	open.add(fromPixel);
	while (open.size > 0) {
		const currentPixel = open.values().next().value;
		open.delete(currentPixel);

		if (blob.size == 0 || sameColor(currentPixel, blobColor)) {
			blob.add(currentPixel);
			pixelList.splice(
				pixelList.findIndex((pix) => pix === currentPixel),
				1
			);

			for (const i of [-1, 0, 1]) {
				for (const j of [-1, 0, 1]) {
					if (i == 0 && j == 0) continue;
					const neighbor = pixelList.find(
						(pix) =>
							pix.x === currentPixel.x + i && pix.y === currentPixel.y + j
					);
					if (neighbor && !blob.has(neighbor)) open.add(neighbor);
				}
			}
		}
	}

	return blob;
}

function frameToPixelList(frame: Frame) {
	const pixelList: PixelList = [];
	const h = frame.length;
	const w = frame[0].length;

	for (let x = 0; x < w; x++) {
		for (let y = 0; y < h; y++) {
			pixelList.push({ ...frame[y][x], x, y });
		}
	}

	return pixelList;
}

function decide(frame: Frame) {
	const blobs = detectBlobs(frame);
	const leftBlob = blobs.find((blob) =>
		sameColor(blob.values().next().value, RED)
	);
	const middleBlob = blobs.find((blob) =>
		sameColor(blob.values().next().value, BLUE)
	);
	const rightBlob = blobs.find((blob) =>
		sameColor(blob.values().next().value, GREEN)
	);

	console.log(
		`Left blob: ${JSON.stringify(
			leftBlob!.values().next().value,
			undefined,
			2
		)}`
	);

	return { throttle: 0, steering: 0 };
}
