namespace dr {

/// Class to filter button events.
class ButtonFilter {
protected:
	/// The old state of the signal.
	bool old_state;

	/// If true, all high signals should pass through.
	bool always_high;

	/// If true, all low signals should pass through.
	bool always_low;

public:
	/// Construct a button filter.
	ButtonFilter(bool always_high = false, bool always_low = false) : always_high(always_high), always_low(always_low) {}

	/// Put the signal through the filter.
	/**
	 * \return True if the signal should be passed on, otherwise false.
	 */
	bool filter(bool state) {
		bool old_state = this->old_state;
		this->old_state = state;

		// If always high is set, high signals should always pass through.
		if (always_high &&  state) return true;

		// If always low is set, low signals should always pass through.
		if (always_low && !state) return true;

		// Otherwise, all edges (rising and falling) should pass through.
		return state != old_state;
	}
};

}
