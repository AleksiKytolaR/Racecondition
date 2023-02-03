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
const GRAY: Color = { r: 122, g: 122, b: 122 };
const PURPLE: Color = { r: 148, g: 0, b: 211 };
const BLACK: Color = { r: 0, g: 0, b: 0 };

const ARROW_SHAPE: Vec2[] = [
	{ x: 0, y: -2 },
	{ x: 0, y: -3 },
	{ x: 0, y: -4 },
	{ x: 0, y: -5 },
	{ x: 0, y: -6 },
];

function getLinePoints(x0: number, y0: number, x1: number, y1: number) {
	const dx = Math.abs(x1 - x0);
	const dy = Math.abs(y1 - y0);
	const sx = x0 < x1 ? 1 : -1;
	const sy = y0 < y1 ? 1 : -1;
	let err = dx - dy;

	const linePoints: Vec2[] = [];
	// eslint-disable-next-line no-constant-condition
	while (true) {
		linePoints.push({ x: x0, y: y0 });
		if (x0 === x1 && y0 === y1) break;
		const e2 = 2 * err;
		if (e2 > -dy) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dx) {
			err += dx;
			y0 += sy;
		}
	}

	return linePoints;
}

class MovingAverageFilter {
	private windowSize: number;
	private values: number[];
	private sum: number;

	constructor(windowSize: number) {
		this.windowSize = windowSize;
		this.values = [];
		this.sum = 0;
	}

	filter(value: number) {
		this.sum += value;
		this.values.push(value);
		if (this.values.length > this.windowSize) {
			this.sum -= this.values.shift()!;
		}

		return this.getAverage();
	}

	getAverage() {
		return this.sum / this.values.length;
	}
}

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
	private minOutput: number;
	private maxOutput: number;
	private minIntegrator: number;
	private maxIntegrator: number;
	public derivativeFiltered = 0;
	public pOut = 0;
	public dOut = 0;
	public iOut = 0;

	constructor(
		kp: number,
		ki: number,
		kd: number,
		derivativeFilterTimeConstant: number,
		minOutput: number,
		maxOutput: number,
		minIntegrator: number,
		maxIntegrator: number
	) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.derivativeFilter = new LowPassFilter(derivativeFilterTimeConstant);
		this.integral = 0;
		this.previousError = 0;
		this.minOutput = minOutput;
		this.maxOutput = maxOutput;
		this.minIntegrator = minIntegrator;
		this.maxIntegrator = maxIntegrator;
	}

	public update(error: number, dt: number): number {
		this.derivativeFiltered = this.derivativeFilter.filter(
			(error - this.previousError) / dt
		);
		this.previousError = error;
		let temp_integrator = this.integral + error * dt * this.ki;
		if (temp_integrator > this.maxIntegrator) {
			temp_integrator = this.maxIntegrator;
		} else if (temp_integrator < this.minIntegrator) {
			temp_integrator = this.minIntegrator;
		}

		this.pOut = this.kp * error;
		this.iOut = temp_integrator;
		this.dOut = this.kd * this.derivativeFiltered;
		let output = this.pOut + this.iOut + this.dOut;

		// Output limits and integrator anti-windup

		let integration_allowed = true;
		if (output > this.maxOutput) {
			output = this.maxOutput;
			console.log('Max', this.maxOutput);
			if (error > 0) {
				integration_allowed = false;
			}
		} else if (output < this.minOutput) {
			output = this.minOutput;
			console.log('Min', this.minOutput);
			if (error < 0) {
				integration_allowed = false;
			}
		}
		if (integration_allowed) {
			this.integral = temp_integrator;
		}

		return output;
	}
}

function findTargetPoint(middleBlob: Pixel[]) {
	const tip = findTipOfBlob(middleBlob, 'FURTHEST_FROM_CENTER');
	const middle = findTopCenterOfBlob(middleBlob);
	return { x: tip.x * 0.6 + middle.x * 0.4, y: tip.y * 0.6 + middle.y * 0.4 };

	//return { x: (tip.x + middle.x) / 2, y: (tip.y + middle.y) / 2 };
}

function sameColor(x: Color, y: Color) {
	return x.r === y.r && x.g === y.g && x.b === y.b;
}

function detectBlobs(frame: Frame, minBlobSize = 35) {
	const h = frame.length;
	const w = frame[0].length;
	const blobs: Pixel[][] = [];
	const pixelList = frameToBluePixelList(frame);

	let count = 0;
	while (pixelList.length > 0) {
		const current = pixelList[0];
		count++;
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

		const matches = blob.size == 0 || sameColor(currentPixel, blobColor);
		if (matches) {
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

function frameToBluePixelList(frame: Frame) {
	const pixelList: PixelList = [];
	const h = frame.length;
	const w = frame[0].length;

	for (let x = 0; x < w; x++) {
		for (let y = 0; y < h; y++) {
			if (!sameColor(frame[y][x], BLUE)) continue;
			pixelList.push({ ...frame[y][x], x, y });
		}
	}

	return pixelList;
}

function findTipOfBlob(
	blob: Pixel[],
	mode: 'LEFT' | 'RIGHT' | 'FURTHEST_FROM_CENTER'
) {
	const blobAverageCoordinate: Vec2 = { x: 0, y: 0 };
	if (mode === 'FURTHEST_FROM_CENTER') {
		for (const pix of blob) {
			blobAverageCoordinate.x += pix.x;
			blobAverageCoordinate.y += pix.y;
		}
		blobAverageCoordinate.x /= blob.length;
		blobAverageCoordinate.y /= blob.length;
	}

	let tip: Pixel = blob[0];
	for (const pix of blob) {
		// First choose highest Y coordinate with a threshold of 3
		if (pix.y - tip.y <= -3) {
			tip = pix;
			continue;
		} else {
			if (
				mode == 'FURTHEST_FROM_CENTER' &&
				Math.abs(pix.x - blobAverageCoordinate.x) >
					Math.abs(tip.x - blobAverageCoordinate.x) &&
				pix.y - tip.y <= 0
			) {
				tip = pix;
			} else if (mode == 'LEFT' && pix.x < tip.x) {
				tip = pix;
			} else if (mode == 'RIGHT' && pix.x > tip.x) {
				tip = pix;
			}
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

let currentTurn = 'S';
let detectedTurn = 'S';
interface TurnHistoryEntry {
	turn: string;
	duration: number;
}
const turnHistory: TurnHistoryEntry[] = [];
let timeSpentInDetectedTurn = 0;
let timeSpentInCurrentTurn = 0;
function turningTracker(steering: number, dt: number) {
	const avgSteer = avgSteerTracker.filter(steering);
	const previousDetectedTurn = detectedTurn;
	detectedTurn = 'S';
	if (avgSteer > TURN_DETECTION_STEER_THRESHOLD) {
		detectedTurn = 'L';
	} else if (avgSteer < -TURN_DETECTION_STEER_THRESHOLD) {
		detectedTurn = 'R';
	}
	if (previousDetectedTurn === detectedTurn) {
		timeSpentInDetectedTurn += dt;
	} else {
		timeSpentInDetectedTurn = 0;
	}

	timeSpentInCurrentTurn += dt;
	if (
		timeSpentInDetectedTurn > TURN_DETECTION_TIME_TRESHOLD &&
		currentTurn !== detectedTurn
	) {
		// "Actual" new turn detected, no noise here
		turnHistory.push({
			turn: currentTurn,
			duration: timeSpentInCurrentTurn,
		});
		if (turnHistory.length > TURN_HISTORY_LENGTH) {
			turnHistory.shift();
		}
		timeSpentInCurrentTurn = 0;
		currentTurn = detectedTurn;
	}

	return currentTurn;
}

function predictTurnsViaHistory() {
	if (turnHistory.length < 12) return null;
	const prevTurns = turnHistory.slice(-5);

	for (let i = 0; i < turnHistory.length - 5; i++) {
		let matches = true;
		for (let j = 0; j < prevTurns.length; j++) {
			if (turnHistory[i + j].turn !== prevTurns[j].turn) {
				matches = false;
				break;
			}
		}

		if (matches) {
			return [
				turnHistory[i + prevTurns.length],
				turnHistory[i + prevTurns.length + 1],
			];
		}
	}

	return null;
}

const MAX_STEERING_AMOUNT = 1;
const THROTTLE_BASE = 0.095;
const THROTTLE_INCREMENT = 0.2;
const THROTTLE_INCREMENT_STEER_DROPOFF_RANGE = 0.45;
const KP = 1 / 120;
const KI = 0;
const KD = 1 / 6000;
const D_LOWPASS_TC = 1 / 10;
const INTEGRATOR_MAX_FACTOR = 0.25;
const TARGET_POSITION_LPF_TC = 1;
const THROTTLE_LPF_TC = 1 / 4;
const ANGLE_CALCULATION_COORDINATE_HEIGHT = 10;
const targetPositionXFilter = new LowPassFilter(TARGET_POSITION_LPF_TC, 32);
const targetPositionYFilter = new LowPassFilter(TARGET_POSITION_LPF_TC, 20);
const throttleFilter = new LowPassFilter(THROTTLE_LPF_TC, 0);
const TURN_DETECTION_STEER_THRESHOLD = 0.3;
const TURN_DETECTION_AVG_FILTER_WINDOW = 6;
const TURN_DETECTION_TIME_TRESHOLD = 0.2;
const TURN_HISTORY_LENGTH = 30;

const steeringPID = new PIDController(
	KP,
	KI,
	KD,
	D_LOWPASS_TC,
	-MAX_STEERING_AMOUNT,
	MAX_STEERING_AMOUNT,
	-MAX_STEERING_AMOUNT * INTEGRATOR_MAX_FACTOR,
	MAX_STEERING_AMOUNT * INTEGRATOR_MAX_FACTOR
);

const avgThrottleTracker = new MovingAverageFilter(6);
const avgSteerTracker = new MovingAverageFilter(6);
let previousTimestamp = Date.now();
let targetCoord: Vec2 = { x: 32, y: 20 };
let targetPointFiltered = targetCoord;
let targetBearingDeg = 0;
let predTurns: TurnHistoryEntry[] | null = null;

function preprocess(frame: Frame) {
	const h = frame.length;
	const w = frame[0].length;
	const threshold = 10;

	const processed = frame.map((row, y) =>
		row.map((pixel, x) => {
			if (y < 13) return PURPLE;
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
		targetCoord = findTargetPoint(middleBlob);
	} else {
		targetCoord.y += 1;
	}

	targetPointFiltered = {
		x: targetPositionXFilter.filter(targetCoord.x),
		y: targetPositionYFilter.filter(targetCoord.y),
	};

	const angleOriginCoordinate: Vec2 = {
		x: frame[0].length / 2,
		y: frame.length - ANGLE_CALCULATION_COORDINATE_HEIGHT,
	};
	const angleTowardsTargetRad = Math.atan2(
		targetPointFiltered.y - angleOriginCoordinate.y,
		targetPointFiltered.x - angleOriginCoordinate.x
	);
	// Calculate bearing towards target, between -180 to 180 deg
	targetBearingDeg = 90 + angleTowardsTargetRad * (180 / Math.PI);
	if (targetBearingDeg > 180) {
		targetBearingDeg -= 360;
	}
	if (targetBearingDeg < -180) {
		targetBearingDeg += 360;
	}

	// Render raw non-LPF target
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
	const lineEnd: Vec2 = {
		x: Math.round(
			angleOriginCoordinate.x + Math.cos(angleTowardsTargetRad) * 55
		),
		y: Math.round(
			angleOriginCoordinate.y + Math.sin(angleTowardsTargetRad) * 55
		),
	};

	const linePoints = getLinePoints(
		angleOriginCoordinate.x,
		angleOriginCoordinate.y,
		lineEnd.x,
		lineEnd.y
	);

	// Render line representing calculated angle towards turn
	for (const pt of linePoints) {
		if (pt.x >= 0 && pt.x < w && pt.y >= 0 && pt.y < h) {
			processed[pt.y][pt.x] = WHITE;
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

function decide(frame: Frame) {
	const timeStamp = Date.now();
	const dt = (timeStamp - previousTimestamp) / 1000;
	previousTimestamp = timeStamp;
	let steering = steeringPID.update(-targetBearingDeg, dt);

	let throttle =
		THROTTLE_BASE +
		THROTTLE_INCREMENT *
			(1 -
				Math.min(
					1,
					Math.abs(steering) /
						(THROTTLE_INCREMENT_STEER_DROPOFF_RANGE * MAX_STEERING_AMOUNT)
				));

	throttle = Math.min(1, Math.max(0, throttle));
	let throttleFiltered = throttleFilter.filter(throttle);
	steering = Math.min(
		MAX_STEERING_AMOUNT,
		Math.max(-MAX_STEERING_AMOUNT, steering)
	);
	avgThrottleTracker.filter(throttle);
	const estSpeed = avgThrottleTracker.getAverage() * 1010 + 64;
	turningTracker(steering, dt);

	predTurns = predictTurnsViaHistory();

	let boosting = false;
	if (predTurns) {
		const nextIsStraight = predTurns[1].turn == 'S';
		const predictedDuration = predTurns[0].duration + predTurns[1].duration;
		const boostCondition1 =
			timeSpentInCurrentTurn > 0.35 * predTurns[0].duration;
		const boostCondition2 = timeSpentInCurrentTurn < 0.7 * predictedDuration;
		const boostCondition3 = predictedDuration > 1.8;
		const canBoost =
			nextIsStraight && boostCondition1 && boostCondition2 && boostCondition3;
		if (canBoost) {
			throttleFiltered *= 3.5;
			boosting = true;
		}
	}

	return {
		throttle: Math.min(1, Math.max(0, throttleFiltered)),
		avgThrottle: avgThrottleTracker.getAverage(),
		currentTurn,
		boosting,
		turnHistory: turnHistory
			.slice(-4)
			.map((t) => `${t.duration.toFixed(2)} ${t.turn}`)
			.join(', '),
		predTurn: predTurns
			? predTurns.map((t) => `${t.duration.toFixed(2)} ${t.turn}`).join(', ')
			: 'N/A',
		timeSpentInCurrentTurn: timeSpentInDetectedTurn,
		estSpeed,
		steering,
		targetBearingDeg,
		pValue: steeringPID.pOut,
		iValue: steeringPID.iOut,
		dValue: steeringPID.dOut,
		dt,
	};
}
