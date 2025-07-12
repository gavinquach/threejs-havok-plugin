/* eslint-disable @typescript-eslint/no-explicit-any */

/**
 * Represents a single subscription (an observer) to an Observable.
 * It holds the callback function and allows for its removal.
 * @template T The type of data sent during notification.
 */
export class Observer<T> {
    /** The callback function to be executed. */
    public readonly callback: (eventData: T) => void;
    /** A flag indicating if the callback should be run only once. */
    public readonly isOnce: boolean;
    /** The scope (this context) to be used when calling the callback. */
    private readonly scope: any;

    /** A private reference to the observable that created this observer. */
    private _observable: Observable<T>;

    /**
     * Creates an instance of an Observer.
     * @param callback The function to execute on notification.
     * @param observable The observable that this observer belongs to.
     * @param isOnce Defines if the observer should be removed after its first execution.
     * @param scope The `this` context for the callback.
     */
    constructor(
        callback: (eventData: T) => void,
        observable: Observable<T>,
        isOnce = false,
        scope?: any,
    ) {
        this.callback = callback;
        this._observable = observable;
        this.isOnce = isOnce;
        this.scope = scope;
    }

    /**
     * Executes the observer's callback function.
     * @param eventData The data to be passed to the callback.
     * @internal
     */
    public _execute(eventData: T): void {
        this.callback.call(this.scope, eventData);
    }

    /**
     * Removes this observer from its parent Observable, unsubscribing it
     * from future notifications.
     */
    public remove(): void {
        this._observable.remove(this);
    }
}

/**
 * A class that implements the Observer pattern. It allows objects to subscribe
 * to notifications and be notified when an event occurs.
 * @template T The type of data sent during notification.
 */
export class Observable<T> {
    private _observers: Observer<T>[] = [];

    /**
     * Adds a new observer to the list.
     * @param callback The function to be called when the observable is notified.
     * @param scope The `this` context to be used for the callback.
     * @returns The newly created Observer instance, which can be used to remove the subscription.
     */
    public add(
        callback: (eventData: T) => void,
        scope: any = undefined,
    ): Observer<T> {
        const observer = new Observer(callback, this, false, scope);
        this._observers.push(observer);
        return observer;
    }

    /**
     * Adds a new observer that will be executed only once. After its first
     * execution, it will be automatically removed.
     * @param callback The function to be called when the observable is notified.
     * @param scope The `this` context to be used for the callback.
     * @returns The newly created Observer instance.
     */
    public addOnce(
        callback: (eventData: T) => void,
        scope: any = undefined,
    ): Observer<T> {
        const observer = new Observer(callback, this, true, scope);
        this._observers.push(observer);
        return observer;
    }

    /**
     * Removes a specific observer from the list.
     * @param observer The observer to remove.
     * @returns `true` if the observer was found and removed, otherwise `false`.
     */
    public remove(observer: Observer<T>): boolean {
        const index = this._observers.indexOf(observer);
        if (index !== -1) {
            this._observers.splice(index, 1);
            return true;
        }
        return false;
    }

    /**
     * Removes all observers that are associated with a specific callback function.
     * @param callback The callback function to remove observers for.
     * @returns The number of observers removed.
     */
    public removeCallback(callback: (eventData: T) => void): number {
        let removedCount = 0;
        this._observers = this._observers.filter((obs) => {
            if (obs.callback === callback) {
                removedCount++;
                return false;
            }
            return true;
        });
        return removedCount;
    }

    /**
     * Notifies all registered observers by executing their callback functions.
     * @param eventData The data to pass to each observer's callback.
     */
    public notifyObservers(eventData: T): void {
        // Create a copy of the observers array to prevent issues if an observer
        // is removed during the notification loop (e.g., in an addOnce callback).
        const observersToNotify = [...this._observers];

        for (const observer of observersToNotify) {
            observer._execute(eventData);

            // If it's a "once" observer, remove it from the original array.
            if (observer.isOnce) {
                this.remove(observer);
            }
        }
    }

    /**
     * Checks if the observable has any registered observers.
     * @returns `true` if there is at least one observer, otherwise `false`.
     */
    public hasObservers(): boolean {
        return this._observers.length > 0;
    }

    /**
     * Removes all observers from this observable.
     */
    public clear(): void {
        this._observers = [];
    }
}