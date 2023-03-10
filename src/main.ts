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

class MaxIncreaseFilter {
	private maxRate: number;
	private previousOutput: number;
	private desiredValue: number;

	constructor(maxRate: number) {
		this.maxRate = maxRate;
		this.previousOutput = 0;
		this.desiredValue = 0;
	}

	public filter(input: number, dt: number) {
		this.desiredValue = input;
		const delta = (this.desiredValue - this.previousOutput) / dt;

		if (delta > this.maxRate) {
			const output = this.previousOutput + this.maxRate * dt;
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

const MAX_STEERING_AMOUNT = 1;
const THROTTLE_BASE = 0.095;
const THROTTLE_INCREMENT = 0.45;
const THROTTLE_INCREMENT_STEER_DROPOFF_RANGE = 0.45;
const TOP_ESTIMATED_SPEED_LIMIT = 290;
const KP = 0.75;
const KI = 0;
const KD = 1 / 60;
const D_LOWPASS_TC = 1 / 2;
const INTEGRATOR_MAX_FACTOR = 0.25;
const avgThrottleTracker = new MovingAverageFilter(6);
const throttleOutputFilter = new MaxIncreaseFilter(999);

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

function decide(frame: Frame) {
	const timeStamp = Date.now();
	const dt = (timeStamp - previousTimestamp) / 1000;
	previousTimestamp = timeStamp;
	const detectedTurn = calculateTurn(frame);
	const steering = steeringPID.update(detectedTurn, dt);
	let throttle =
		THROTTLE_BASE +
		THROTTLE_INCREMENT *
			(1 -
				Math.min(
					1,
					Math.abs(steering) /
						(THROTTLE_INCREMENT_STEER_DROPOFF_RANGE * MAX_STEERING_AMOUNT)
				));

	const estSpeed = avgThrottleTracker.getAverage() * 1010 + 64;

	const throttleLimit = 0.05 + (estSpeed / 250) * 0.15;

	if (estSpeed > TOP_ESTIMATED_SPEED_LIMIT) {
		throttle *= 0;
	}

	if (throttle > throttleLimit) throttle = throttleLimit;
	throttle = Math.min(1, Math.max(0, throttle));

	avgThrottleTracker.filter(throttle);

	return {
		throttle,
		max_throttle: throttleLimit,
		estSpeed,
		steering,
		detectedTurn,
		pOut: steeringPID.pOut,
		iOut: steeringPID.iOut,
		dOut: steeringPID.dOut,
	};
}
