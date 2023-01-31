import { ComponentFixture, TestBed } from '@angular/core/testing';

import { OverviewControlLoopComponent } from './overview-control-loop.component';

describe('OverviewControlLoopComponent', () => {
  let component: OverviewControlLoopComponent;
  let fixture: ComponentFixture<OverviewControlLoopComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ OverviewControlLoopComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(OverviewControlLoopComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
