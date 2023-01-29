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
const WHITE: Color = { r: 255, g: 255, b: 255 };
const GRAY: Color = { r: 100, g: 100, b: 100 };
const BLACK: Color = { r: 0, g: 0, b: 0 };

const ARROW_SHAPE: Vec2[] = [
	{ x: 0, y: -2 },
	{ x: 0, y: -3 },
	{ x: 0, y: -4 },
	{ x: 0, y: -5 },
	{ x: 0, y: -6 },
];

class LowPassFilter {
	private alpha: number;
	private previousOutput: number;

	constructor(timeConstant: number, initialOutput = 0) {
		this.alpha = 1 / (timeConstant + 1);
		this.previousOutput = initialOutput;
	}

	public filter(input: number): number {
		const output = this.alpha * input + (1 - this.alpha) * this.previousOutput;
		this.previousOutput = output;
		return output;
	}
}

class PIDController {
	private kp: number;
	private ki: number;
	private kd: number;
	private derivativeFilter: LowPassFilter;
	private integral = 0;
	private previousError: number;
	private maxIntegral: number;
	public derivativeFiltered = 0;
	public pValue = 0;
	public dValue = 0;
	public iValue = 0;

	constructor(
		kp: number,
		ki: number,
		kd: number,
		derivativeFilterTimeConstant: number,
		maxIntegral: number
	) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.derivativeFilter = new LowPassFilter(derivativeFilterTimeConstant);
		this.integral = 0;
		this.previousError = 0;
		this.maxIntegral = maxIntegral;
	}

	public update(error: number, dt: number): number {
		this.integral += error * dt * this.ki;
		this.integral = Math.max(
			-this.maxIntegral,
			Math.min(this.integral, this.maxIntegral)
		);
		this.derivativeFiltered = this.derivativeFilter.filter(
			(error - this.previousError) / dt
		);
		this.previousError = error;
		this.pValue = this.kp * error;
		this.iValue = this.integral;
		this.dValue = this.kd * this.derivativeFiltered;
		return this.pValue + this.iValue + this.dValue;
	}
}

const MAX_STEERING_AMOUNT = 1;
const steeringPID = new PIDController(
	1.0 / 45,
	0, //1 / 1000,
	7 / 1000,
	1.0 / 3,
	MAX_STEERING_AMOUNT
);
const targetPositionXLPF = new LowPassFilter(1 / 2, 0);
const targetPositionYLPF = new LowPassFilter(1 / 2, 0);
let previousTimestamp = Date.now();
let targetCoord: Vec2 = { x: 32, y: 20 };
let targetPointFiltered = targetCoord;

function preprocess(frame: Frame) {
	const h = frame.length;
	const w = frame[0].length;
	const threshold = 10;
	const processed = frame.map((row, y) =>
		row.map((pixel, x) => {
			if (pixel.r - Math.max(pixel.g, pixel.b) > threshold) return RED;
			if (pixel.g - Math.max(pixel.r, pixel.b) > threshold) return GREEN;
			if (pixel.b - Math.max(pixel.g, pixel.r) > threshold) return BLUE;
			return BLACK;
		})
	);

	const blobs = detectBlobs(processed);
	const middleBlob = blobs
		.filter((blob) => sameColor(blob[0], BLUE))
		.sort((a, b) => a.length - b.length)[0];

	if (middleBlob) {
		// If blob found, use new detected tip. Otherwise use previous value
		targetCoord = findTopCenterOfBlob(middleBlob);
	}

	targetPointFiltered = {
		x: targetPositionXLPF.filter(targetCoord.x),
		y: targetPositionYLPF.filter(targetCoord.y),
	};

	// Render raw target
	for (const arrowOffset of ARROW_SHAPE) {
		const point: Vec2 = {
			x: Math.round(targetCoord.x + arrowOffset.x),
			y: Math.round(targetCoord.y + arrowOffset.y),
		};
		if (point.x >= 0 && point.x < w && point.y >= 0 && point.y < h) {
			try {
				processed[point.y][point.x] = GRAY;
			} catch (e) {
				console.log(point);
			}
		}
	}

	// Render filtered target
	for (const arrowOffset of ARROW_SHAPE) {
		const point: Vec2 = {
			x: Math.round(targetPointFiltered.x + arrowOffset.x),
			y: Math.round(targetPointFiltered.y + arrowOffset.y),
		};
		if (point.x >= 0 && point.x < w && point.y >= 0 && point.y < h) {
			try {
				processed[point.y][point.x] = WHITE;
			} catch (e) {
				console.log(point);
			}
		}
	}
	return processed;
}

function sameColor(x: Color, y: Color) {
	return x.r === y.r && x.g === y.g && x.b === y.b;
}

function detectBlobs(frame: Frame, minBlobSize = 35) {
	const h = frame.length;
	const w = frame[0].length;
	const blobs: Pixel[][] = [];
	const pixelList = frameToPixelList(frame);

	while (pixelList.length > 0) {
		const current = pixelList[0];
		const blob = detectBlobFromPoint(pixelList, current);
		if (blob.length < minBlobSize) continue;
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

	return Array.from(blob);
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

function findTipOfBlob(blob: Pixel[]) {
	const blobAverageCoordinate: Vec2 = { x: 0, y: 0 };
	for (const pix of blob) {
		blobAverageCoordinate.x += pix.x;
		blobAverageCoordinate.y += pix.y;
	}
	blobAverageCoordinate.x /= blob.length;
	blobAverageCoordinate.y /= blob.length;

	let tip: Pixel = blob[0];
	for (const pix of blob) {
		// First choose highest Y coordinate with a threshold of 3
		if (pix.y - tip.y <= -3) {
			tip = pix;
			continue;
		} else if (
			// Second choose highest distance from blob average X coord
			Math.abs(pix.x - blobAverageCoordinate.x) >
				Math.abs(tip.x - blobAverageCoordinate.x) &&
			pix.y - tip.y <= 0
		) {
			tip = pix;
			continue;
		}
	}
	return tip;
}

function findTopCenterOfBlob(blob: Pixel[]) {
	let minY = blob[0].y;
	for (const pix of blob) {
		if (pix.y < minY) {
			minY = pix.y;
		}
	}

	const averageTopCoordinate: Vec2 = { x: 0, y: 0 };
	let count = 0;
	for (const pix of blob) {
		if (pix.y <= minY) {
			averageTopCoordinate.x += pix.x;
			averageTopCoordinate.y += pix.y;
			count++;
		}
	}
	averageTopCoordinate.x /= count;
	averageTopCoordinate.y /= count;
	return averageTopCoordinate;
}

function decide(frame: Frame) {
	const centerCoordinate: Vec2 = {
		x: frame[0].length / 2,
		y: frame.length / 2,
	};

	const vectorTowardsMiddle: Vec2 = {
		x: centerCoordinate.x - targetPointFiltered.x,
		y: centerCoordinate.y - targetPointFiltered.y,
	};

	const timeStamp = Date.now();
	const dt = (timeStamp - previousTimestamp) / 1000;
	previousTimestamp = timeStamp;
	let steering = steeringPID.update(vectorTowardsMiddle.x, dt);
	let throttle =
		0.03 +
		0.04 * (1 - Math.min(1, Math.abs(steering) / (0.6 * MAX_STEERING_AMOUNT)));

	throttle = Math.min(1, Math.max(0, throttle));
	steering = Math.min(
		MAX_STEERING_AMOUNT,
		Math.max(-MAX_STEERING_AMOUNT, steering)
	);

	return {
		throttle,
		vectorTowardsMiddle,
		steering,
		pValue: steeringPID.pValue,
		iValue: steeringPID.iValue,
		dValue: steeringPID.dValue,
		dt,
	};
}
