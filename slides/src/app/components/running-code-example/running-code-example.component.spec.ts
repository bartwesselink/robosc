import { ComponentFixture, TestBed } from '@angular/core/testing';

import { RunningCodeExampleComponent } from './running-code-example.component';

describe('RunningCodeExampleComponent', () => {
  let component: RunningCodeExampleComponent;
  let fixture: ComponentFixture<RunningCodeExampleComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ RunningCodeExampleComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(RunningCodeExampleComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
