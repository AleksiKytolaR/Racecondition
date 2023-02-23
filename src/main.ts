/* eslint-disable @typescript-eslint/no-non-null-assertion */
/* eslint-disable @typescript-eslint/no-unused-vars */
type Frame = Color[][];
type Color = { r: number; g: number; b: number };
type Vec2 = { x: number; y: number };
type Pixel = Vec2 & Color;
const RED: Color = { r: 255, g: 0, b: 0 };
const GREEN: Color = { r: 0, g: 255, b: 0 };
const BLUE: Color = { r: 0, g: 0, b: 255 };
const WHITE: Color = { r: 255, g: 255, b: 255 };
const GRAY: Color = { r: 122, g: 122, b: 122 };
const PURPLE: Color = { r: 148, g: 0, b: 211 };
const BLACK: Color = { r: 0, g: 0, b: 0 };

interface SamplingCurvePoint {
	x: number;
	y: number;
}

class SamplingCurve {
	points_: SamplingCurvePoint[];
	constructor(points: SamplingCurvePoint[]) {
		this.points_ = points;
		this.points_.sort((a, b) => a.x - b.x);
		console.log(this.points_);
	}

	getValue(x: number) {
		let previousPoint: SamplingCurvePoint | undefined;
		let nextPoint: SamplingCurvePoint | undefined;

		for (let i = 0; i < this.points_.length - 1; i++) {
			const p = this.points_[i];
			const pPlusOne = this.points_[i + 1];
			if (p.x <= x && (!previousPoint || p.x > previousPoint.x)) {
				previousPoint = p;
				nextPoint = pPlusOne;
			}
		}

		if (!previousPoint || !nextPoint) {
			throw new Error('Sampled point not on curve!');
		}
		const dx = nextPoint.x - previousPoint.x;
		const dy = nextPoint.y - previousPoint.y;
		const output = previousPoint.y + dy * ((x - previousPoint.x) / dx);
		return output;
	}
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

class MaxChangeFilter {
	private maxIncrease: number | undefined;
	private maxDecrease: number | undefined;
	private previousOutput: number;
	private desiredValue: number;

	constructor(maxIncrease?: number, maxDecrease?: number) {
		this.maxIncrease = maxIncrease;
		this.maxDecrease = maxDecrease;
		this.previousOutput = 0;
		this.desiredValue = 0;
	}

	public filter(input: number, dt: number) {
		this.desiredValue = input;
		const delta = (this.desiredValue - this.previousOutput) / dt;

		if (this.maxIncrease !== undefined && delta > this.maxIncrease) {
			const output = this.previousOutput + this.maxIncrease * dt;
			this.previousOutput = output;
			return output;
		}
		if (this.maxDecrease !== undefined && delta < this.maxDecrease) {
			const output = this.previousOutput + this.maxDecrease * dt;
			this.previousOutput = output;
			return output;
		}

		this.previousOutput = this.desiredValue;
		return this.desiredValue;
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
			if (error > 0) {
				integration_allowed = false;
			}
		} else if (output < this.minOutput) {
			output = this.minOutput;
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

function sameColor(x: Color, y: Color) {
	return x.r === y.r && x.g === y.g && x.b === y.b;
}

// PID Tuning Parameters
const steeringDampener = new MaxChangeFilter(12.5, -12.5);
const MAX_STEERING_AMOUNT = 1;
const KP = 0.8;
const KI = 0;
const KD = 1 / 90;
const D_LOWPASS_TC = 1 / 2;
const INTEGRATOR_MAX_FACTOR = 0.25;

// Throttle tuning
const steerTrackingLpf = new LowPassFilter(1 / 2);
const throttleFilter = new MaxChangeFilter(0.55, undefined);
const throttleCurve = new SamplingCurve([
	{
		x: 0,
		y: 0.3,
	},
	{
		x: 0.2,
		y: 0.23,
	},
	{
		x: 0.4,
		y: 0.17,
	},
	{
		x: 0.6,
		y: 0.12,
	},
	{
		x: 0.8,
		y: 0.092,
	},
	{
		x: 1,
		y: 0.09,
	},
]);

// Turn detection
const TURN_DETECTION_STEER_THRESHOLD = 0.275;
const TURN_DETECTION_TIME_TRESHOLD = 0.25;
const TURN_HISTORY_LENGTH = 35;
const avgSteerTracker = new MovingAverageFilter(6);
const turnDetectionLpf = new LowPassFilter(1 / 6);

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

let previousTimestamp = Date.now();
let currentTurn = 'S';
let detectedTurn = 'S';
interface TurnHistoryEntry {
	turn: string;
	duration: number;
}
const turnHistory: TurnHistoryEntry[] = [];
let predictedTurns: TurnHistoryEntry[] | null = null;
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
	const prevTurns = turnHistory.slice(-6);

	for (let i = turnHistory.length - 6 - 4; i >= 0; i--) {
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

function preprocess(frame: Frame) {
	const threshold = 10;
	const firstProcessed = frame.map((row, y) =>
		row.map((pixel, x) => {
			if (y < 13) return PURPLE;
			if (pixel.r - Math.max(pixel.g, pixel.b) > threshold) return RED;
			if (pixel.g - Math.max(pixel.r, pixel.b) > threshold) return GREEN;
			if (pixel.b - Math.max(pixel.g, pixel.r) > threshold) return BLUE;
			return BLACK;
		})
	);

	const processed = firstProcessed.map((row, y) =>
		row.map((pixel, x) => {
			try {
				const below = firstProcessed[y + 1][x];
				if (sameColor(pixel, GREEN) && sameColor(below, RED)) {
					return RED;
				} else if (sameColor(pixel, RED) && sameColor(below, GREEN)) {
					return GREEN;
				} else {
					return pixel;
				}
			} catch (e) {
				return pixel;
			}
		})
	);

	return processed;
}

const isRed = (c: Color) => sameColor(c, RED);
const isBlue = (c: Color) => sameColor(c, BLUE);
const isGreen = (c: Color) => sameColor(c, GREEN);
const hasRedOrGreen = (row: Color[]) =>
	row.filter((c) => isRed(c) || isGreen(c)).length > 5;

let lastTurn = 0;
const calculateTurn = (frame: Frame) => {
	const firstRowIndex = frame.findIndex(hasRedOrGreen);
	const firstRowsPixels = frame
		.slice(firstRowIndex, firstRowIndex + 4)
		.flat()
		.filter((c) => isRed(c) || isGreen(c));

	const total = firstRowsPixels.length;
	if (total === 0) return lastTurn;
	const redsPercentage = firstRowsPixels.filter(isRed).length / total;
	const greensPercentage = firstRowsPixels.filter(isGreen).length / total;
	const turn = greensPercentage - redsPercentage;
	lastTurn = turn;
	return turn;
};

const calculateThrottle = (steering: number, dt: number, boost: boolean) => {
	const steeringFiltered = steerTrackingLpf.filter(steering);
	let throttle = throttleCurve.getValue(Math.abs(steeringFiltered));
	if (boost) {
		throttle += 0.03;
	}
	const filteredThrottle = throttleFilter.filter(throttle, dt);
	return filteredThrottle;
};

const shouldBoost = () => {
	if (
		predictedTurns &&
		predictedTurns[0].turn === 'S' &&
		timeSpentInCurrentTurn < predictedTurns[0].duration * 0.45 &&
		predictedTurns[0].duration > 1.2
	) {
		return true;
	}
	if (
		predictedTurns &&
		predictedTurns[1].turn === 'S' &&
		timeSpentInCurrentTurn > predictedTurns[0].duration * 0.35 &&
		predictedTurns[1].duration > 0.8 &&
		predictedTurns[0].duration + predictedTurns[1].duration > 2.4
	) {
		return true;
	}

	return false;
};

const calculateDrivingOffset = () => {
	let offset = 0;
	if (
		predictedTurns &&
		predictedTurns[0].turn === 'S' &&
		predictedTurns[0].duration > 0.65 &&
		timeSpentInCurrentTurn > 0.25 &&
		predictedTurns[0].duration - timeSpentInCurrentTurn > 0.35
	) {
		if (predictedTurns[1].turn === 'R') {
			offset = 0.04;
		} else {
			offset = -0.04;
		}
	}
	if (
		predictedTurns &&
		predictedTurns[0].turn !== 'S' &&
		predictedTurns[1].turn === 'S' &&
		timeSpentInCurrentTurn > predictedTurns[0].duration * 0.25
	) {
		if (predictedTurns[0].turn === 'R') {
			offset = 0.02;
		} else {
			offset = -0.02;
		}
	}
	return offset;
};

function decide(frame: Frame) {
	const timeStamp = Date.now();
	const dt = (timeStamp - previousTimestamp) / 1000;
	previousTimestamp = timeStamp;

	const detectedTurn = turnDetectionLpf.filter(calculateTurn(frame));
	predictedTurns = predictTurnsViaHistory();

	let steering = steeringPID.update(detectedTurn, dt);
	const offset = 0; //calculateDrivingOffset();;
	steering += offset;
	const steeringDampened = steeringDampener.filter(steering, dt);
	const boost = shouldBoost();
	const throttle = calculateThrottle(steeringDampened, dt, boost);

	avgSteerTracker.filter(steeringDampened);
	turningTracker(avgSteerTracker.getAverage(), dt);

	return {
		throttle,
		boost,
		offset,
		steering: steeringDampened,
		currentTurn,
		timeSpentInCurrentTurn,
		turnHistory: turnHistory
			.slice(-4)
			.map((t) => `${t.duration.toFixed(1)} ${t.turn}`)
			.join(', '),
		predictedTurns: predictedTurns
			?.slice(-4)
			.map((t) => `${t.duration.toFixed(1)} ${t.turn}`)
			.join(', '),
		pOut: steeringPID.pOut,
		iOut: steeringPID.iOut,
		dOut: steeringPID.dOut,
	};
}
